#!/usr/bin/env python3
# example.py
"""
Run a quick demo on a real C source file.

Usage
-----
    python -m c_struct_parser.example <path/to/c/file.c>
"""

import argparse

from tools.lib.python.kdoc.data_parser import CDataParser

TEST = """
typedef
struct property_entry {
	const char *name;
	size_t length;
	bool is_inline;   /* TEST */
	struct foo {
		char *bar[12];
		struct foobar {
			enum enum_type my_enum; /* TEST	2 */
			struct {
				uint_t test; /* TEST 3 */
				static const int anonymous;
			};
		};
		;;        /* This is valid, but should not occur in practice */
		{};       /* Same here */
	};
	enum dev_prop_type type;
		enum {
			EXPRESSION_LITERAL,
			EXPRESSION_BINARY,
			EXPRESSION_UNARY,
			EXPRESSION_FUNCTION,
			EXPRESSION_ARRAY
		} literal;

	union {
		const void *pointer;
		union {
			u8 boou8_data[sizeof(u64) / sizeof(u8)];
			u16 u16_data[sizeof(u64) / sizeof(u16)];
			u32 u32_data[sizeof(u64) / sizeof(u32)];
			u64 u64_data[sizeof(u64) / sizeof(u64)];
			const char *str[sizeof(u64) / sizeof(char *)];
		};
	};
	char *prop_name;
} my_type;
"""


def main():
    p = argparse.ArgumentParser(description="Parse a C struct/union/enum definition.")

    p.add_argument("fname", nargs="?", help="C source file to parse")

    args = p.parse_args()

    if args.fname:
        with open(args.fname, "r", encoding="utf-8") as f:
            source = f.read()
    else:
        source = TEST

    parser = CDataParser(source)

    item = parser.item

    print(repr(item))
    print()

    if item.typedef_name:
        print(f"typedef {item.typedef_name}\n")

    if item.decl_type and item.decl_name:
        print(f"{item.decl_type} {item.decl_name}\n")

    if item.parameterlist:
        print("parameterlist:")
        for p in item.parameterlist:
            print(f"  - {p}")
        print()

    if item.parametertypes:
        print("parametertypes:")
        for k, v in item.parametertypes.items():
            print(f"  - {k}: {v}")


if __name__ == "__main__":
    main()
