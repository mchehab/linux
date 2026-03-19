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

    def __init__(self, source):
        self.source = source
        self.item = CDataItem()

        IGNORE_TOKENS = [CToken.SPACE, CToken.COMMENT]

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
            if tok.kind not in IGNORE_TOKENS:
                break

        #
        # Special case: handle typedefs
        #
        if tok.kind == CToken.TYPEDEF:
            for j in range(end - 1, i, -1):
                tok = tokens[j]
                if tok.kind not in IGNORE_TOKENS + [CToken.ENDSTMT]:
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
        self.member_start = end     # Initialize with biggest value
        self.member_end = start     # Initialize with lowest value

        self.item.parametertypes = self._parse(tokens, start, end)
        self.item.parameterlist = list(self.item.parametertypes.keys())

        members = CTokenizer(tokens[self.member_start + 1:self.member_end])

        self.item.members = str(members)

        #
        # Special case: handle variable declarations
        #
        if (not self.item.decl_name and len(self.item.parameterlist) == 1
            and len(self.item.parametertypes) == 1):
            self.item.decl_type = "var"
            self.item.decl_name = self.item.parameterlist[0]
            self.item.parameterlist = []

    def out_cur_type(self, token_list, parametertypes):
        #
        # End of an statement. Parse it if tokens are present
        #

        # Drop spaces at the end
        while token_list and token_list[-1].kind == CToken.SPACE:
            token_list.pop()

        if not token_list:
            return

        start_level = token_list[0].level

        #
        # the last NAME token with level 0 is the field name
        #
        tok_names = []
        has_comma = False
        type_end = 0

        for pos, cur_tok in enumerate(token_list):

            if cur_tok.level != start_level:
                continue


            if cur_tok.kind == CToken.COMMA:
                has_comma = True
                pos -= 1
                break

            if cur_tok.kind in [CToken.NAME, CToken.ENUM]:
                type_end = pos - 1
                tok_names.append(cur_tok. value)

        if not tok_names:
            return

        if len(tok_names) < 2:
            print(f"WARNING: missing type or name in {tok_names}. Parse error?")
            return

        var_type = ""
        type_name = ""

        for j in range(0, pos + 1):
            cur_tok = token_list[j]
            value = cur_tok.value

            if cur_tok.kind == CToken.SPACE:
                value = " "

            if j <= type_end:
                var_type += value
            else:
                type_name += value

        var_type = var_type
        type_name = type_name

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
            for j in range(pos + 1, len(token_list)):
                cur_tok = token_list[j]

                if cur_tok.kind == CToken.COMMA:
                    if type_name:
                        type_list.append(type_name.strip())
                        type_name = ""

                    continue

                type_name += cur_tok.value

                if cur_tok.level != start_level:
                    continue

                if cur_tok.kind == CToken.NAME:
                    name_list.append(cur_tok.value)

            if type_name:
                type_list.append(type_name.strip())

        for i in range(0, len(name_list)):
            name = name_list[i]
            parametertypes[name] = var_type

            if i < len(type_list):
                parametertypes[name] += type_list[i]

    def _parse(self, tokens, start, end, parse_level=0):
        """
        Use the tokenizer to handle data structs.
        """

        from textwrap import indent
        if parse_level:
            while start < end and tokens[start].kind == CToken.SPACE:
                start += 1

        token_list = []
        parametertypes = {}

        prev_kind = None
        inner_begin = None
        inner_level = None

        i = start
        while i < end:
            tok = tokens[i]
            if tok.kind == CToken.COMMENT:
                i += 1
                continue

            if tok.kind in [CToken.STRUCT, CToken.UNION] and prev_kind is None:
                prev_kind = i
                i += 1

                while i < end and tokens[i].kind == CToken.SPACE:
                    i += 1

                # Ignore struct name if any

                if i < end and tokens[i].kind == CToken.NAME:
                    if self.item.decl_type and not self.item.decl_name:
                        self.item.decl_name = tok.value

                    i += 1

                continue

            if (tok.kind == CToken.BEGIN and tok.value == "{" and
                prev_kind is not None and inner_begin is None):

                i += 1

                inner_begin = i + 1
                self.member_start = min(self.member_start, i)
                continue

            elif tok.kind == CToken.END and tok.value == "}" and inner_begin is not None:
                i += 1

                if tokens[prev_kind].level != tok.level:
                    token_list.append(tok)
                    continue

                self.member_end = max(self.member_end, i)
                inner_end = i


                while i < end and tokens[i].kind == CToken.SPACE:
                    i += 1

                ptype = tokens[prev_kind].value
                struct_name = ""
                if tokens[i].kind == CToken.NAME:
                    name = tokens[i].value

                    parametertypes[name] = f"{ptype} {name}"
                    struct_name = name  + "."


                    i += 1
                else:
                    name = "{" + f"unnamed_{ptype}" + "}"
                    parametertypes[name] = ptype

                #
                # Recursively handle inner parameters
                #

                new_params = self._parse(tokens, inner_begin, inner_end,
                                         parse_level + 1)

                #
                # Note: names here are unique, as they're keys for
                # parameter types. It means that we need to preserve
                # the struct_name here for unnamed stucts when applicable.
                #
                for name, value in new_params.items():
                    parametertypes[struct_name + name] = value

                token_list=[]
                prev_kind = None
                inner_begin = None

                continue

            i += 1

            if tok.kind != CToken.ENDSTMT or inner_begin:
                if not token_list and tok.kind == CToken.SPACE:
                    continue

                token_list.append(tok)
                continue

            self.out_cur_type(token_list, parametertypes)
            token_list = []

        return parametertypes


