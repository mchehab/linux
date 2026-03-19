#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# Copyright(c) 2025: Mauro Carvalho Chehab <mchehab@kernel.org>.

"""
C lexical parser for variables.
"""

import logging
import re

from .c_lex import CTokenizer, CToken

class CDataItem:
    """
    Represent a data declaration.
    """
    def __init__(self):
        self.typedef_name = None
        self.decl_name = None
        self.decl_type = None
        self.parameterlist = []
        self.parametertypes = {}
        self.members = None

    def __repr__(self) -> str:
        """
        Return contents of the CDataItem.
        Useful for debugging purposes.
        """
        return (f"CDataItem(decl_type={self.decl_type!r}, "
                f"typedef_name={self.typedef_name!r}, "
                f"decl_name={self.decl_name!r}, "
                f"parameterlist={self.parameterlist!r}, "
                f"parametertypes={self.parametertypes!r})")

class CDataParser:
    """
    Handles a C data prototype, converting it into a data element
    describing it.
    """

    IGNORE_TOKENS = [CToken.SPACE, CToken.COMMENT]

    def __init__(self, source):
        self.source = source
        self.item = CDataItem()

        if not isinstance(source, CTokenizer):
            source = CTokenizer(self.source)

        self.log = source.log

        tokens = source.tokens

        start = 0
        end = len(tokens)

        #
        # Ignore initial comments/spaces
        #
        for i in range(0, len(tokens)):
            tok = tokens[i]
            if tok.kind not in self.IGNORE_TOKENS:
                break

        #
        # Special case: handle typedefs
        #
        if tok.kind == CToken.TYPEDEF:
            for j in range(end - 1, i, -1):
                tok = tokens[j]
                if tok.kind not in self.IGNORE_TOKENS + [CToken.ENDSTMT]:
                    break

            if i != j and tok.kind == CToken.NAME:
                self.item.typedef_name = tok.value
                start = i
                end = j
            else:
                self.log.warning("typedef used without a name")

        #
        # Handle the remaining tokens
        #
        self._parse(tokens, start, end)

        #
        # Special case: handle variable declarations
        #
        if (not self.item.decl_name and len(self.item.parameterlist) == 1
            and len(self.item.parametertypes) == 1):
            self.item.decl_type = "var"
            self.item.decl_name = self.item.parameterlist[0]
            self.item.parameterlist = []

    def out_cur_type(self, current_type):
        #
        # End of an statement. Parse it if tokens are present
        #

        if not current_type:
            return None, None

        parameterlist = []
        parametertypes = {}

        #
        # the last NAME token with level 0 is the field name
        #
        tok_names = []
        has_comma = False
        cur_level = 0
        type_end = 0

        for pos, t in enumerate(current_type):
            _, cur_tok = t

            if cur_tok.kind == CToken.BEGIN:
                cur_level += 1
                continue

            if cur_tok.kind == CToken.END:
                cur_level -= 1
                continue

            if cur_level:
                continue

            if cur_tok.kind == CToken.COMMA:
                has_comma = True
                pos -= 1
                break

            if cur_tok.kind == CToken.NAME:
                type_end = pos - 1
                tok_names.append(cur_tok. value)

        if not tok_names:
            return None, None

        var_type = ""
        type_name = ""

        for j in range(0, pos + 1):
            _, cur_tok = current_type[j]

            if j <= type_end:
                var_type += cur_tok.value
            else:
                type_name += cur_tok.value

        var_type = var_type.strip() + " "
        type_name = type_name.strip()

        #
        # The name we'll use is the last name element
        #
        name_list = [ tok_names[-1] ]
        type_list = [ type_name ]

        #
        # Pick remaining elements of the same type
        #
        if has_comma:
            type_name = ""
            for j in range(pos + 1, len(current_type)):
                cur_level, cur_tok = current_type[j]

                if cur_tok.kind == CToken.COMMA:
                    if type_name:
                        type_list.append(type_name.strip())
                        type_name = ""

                    continue

                type_name += cur_tok.value

                if cur_level:
                    continue

                if cur_tok.kind == CToken.NAME:
                    name_list.append(cur_tok.value)

        if type_name:
            type_list.append(type_name.strip())

        for i in range(0, len(name_list)):
            name = name_list[i]

            parameterlist.append(name)
            parametertypes[name] = var_type + type_list[i]

        return parameterlist, parametertypes

    def _parse(self, tokens, start, end):
        """
        Use the tokenizer to handle data structs.
        """

        #
        # Handling struct/union named inner data is complex and require
        # either a recursive algorithm or two passes. To make the code
        # simpler to understand and faster, the logic below uses a multi step
        # approach. The logic here relies on python 3.7+ dicts that preserve
        # the order where elements are added there at structs.
        #

        structs = {}

        #
        # Step 1: parse all elements inside the code, storing them at
        #         structs. Elements are grouped per inner level
        #
        # The key element is a number for unamed dict or a string for
        # named ones.
        #
        current_type = []
        parameters = []

        prev_kind = None
        level = 0
        stack_level = 0
        max_stack_level = 0
        el_id = 0
        last_id = None

        member_start = end
        member_end = start

        for i in range(start, end):
            tok = tokens[i]
            if tok.kind == CToken.COMMENT:
                continue

            if tok.kind in [CToken.STRUCT, CToken.UNION, CToken.ENUM]:
                prev_kind = i

            if tok.kind == CToken.NAME:
                if self.item.decl_type and not self.item.decl_name:
                    self.item.decl_name = tok.value

            if tok.kind == CToken.BEGIN:
                if tok.value == "{":
                    member_start = min(member_start, i)

                    structs[el_id] = {
                        "level": stack_level,
                        "name": None,
                        "begin": i,
                        "parameterlist": [],
                        "parametertypes": {},
                    }

                    last_id = el_id
                    el_id += 1

                    if prev_kind is not None and not self.item.decl_type:
                        self.item.decl_type = tokens[prev_kind].value

                    current_type = []

                    stack_level += 1
                else:
                    level += 1

            elif tok.kind == CToken.END:
                if tok.value == "}":
                    member_end = max(member_end, i)

                    if prev_kind is None:
                        stack_level -= 1
                        continue

                    structs[last_id]["end"] = i

                    current_type = []
                    for j in range(prev_kind, i + 1):
                        current_type.append((level, tokens[j]))
                        if tok.kind == CToken.BEGIN:
                            break

                    j += 1

                    while j < end:
                        if tokens[j].kind not in self.IGNORE_TOKENS:
                            break
                        j += 1

                    if tokens[j].kind == CToken.NAME:
                        name = tokens[j].value

                        structs[last_id]["name"] = name
                        max_stack_level = max(max_stack_level, stack_level)

                    stack_level -= 1

                    prev_kind = None
                else:
                    level -= 1

            elif tok.kind != CToken.ENDSTMT:
                current_type.append((level, tok))
                continue

            parameterlist, parametertypes = self.out_cur_type(current_type)
            current_type = []
            if not parameterlist:
                continue

            structs[last_id]["parameterlist"] += parameterlist
            structs[last_id]["parametertypes"].update(parametertypes)

            continue


        #
        # Step 2: convert element names into names inside the struct/union
        #
        names = [[None] * (max_stack_level + 1) for _ in structs]

        for el_key in reversed(structs.keys()):
            el_begin = structs[el_key]["begin"]
            el_end =  structs[el_key].get("end", end)

            for pos, el in structs.items():
                if pos == el_key or not el["name"]:
                    continue

                if el["begin"] >= el_begin and el["end"] >= el_end:
                    print(f'names[{el_key}][{el["level"]}] = {el["name"]}:')

                    names[pos][el["level"]] = el["name"]

        from pprint import pprint
        print()
        pprint(names)
        pprint
        pprint(structs)

        #
        # Step 3: Merge everything at self.item, using proper C names
        #
        for el_id, el in structs.items():
            named_el = []

            for name in names[el_id]:
                if name:
                    named_el.append(name)

            base_name = "." . join(named_el)

            if base_name:
                base_name += "."

            types = el["parametertypes"]

            for name in el["parameterlist"]:
                full_name = base_name + name

                self.item.parameterlist.append(full_name)
                self.item.parametertypes[full_name] = types[name]

        print(self.item.parameterlist)
        print()
        print(self.item.parametertypes)

        self.item.members = str(CTokenizer(tokens[member_start + 1:member_end]))
