#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# Copyright(c) 2026: Mauro Carvalho Chehab <mchehab@kernel.org>.
#
# pylint: disable=C0413,R0904


"""
Unit tests for kernel-doc CMatch.
"""

import os
import sys
import unittest

# Import Python modules

SRC_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(SRC_DIR, "../lib/python"))

from kdoc.data_parser import CDataParser
from unittest_helper import run_unittest


class TestDataParser(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Ensure that there won't be limit for diffs"""
        cls.maxDiff = None

    def test_parse_named_inner_structs(self):
        source = """
            struct property_entry {
                int a, b, c;
                const char *name;
                u8 auto_seq:1;
                size_t length1[2], length2[5], length3[9];
                bool is_inline;   /* TEST */
                struct {
                    int undescribed_arg;
                    char bol:4;
                } baz;

                struct {
                    char *bar[12];	/* full_name: bar*/
                    struct {
                        struct {
                            struct {
                                enum enum_type my_enum; /* full_name: foo2.my_enum */
                            };
                            struct {
                                uint_t test; /* full_name: foo2.test */
                                static const int ano; /* full_name: foo2.ano */
                            } foo3;
                        } foo2 ;
                    }
                    ;;
                    {};
                } fafafa;
                enum dev_prop_type type; /* full_name: type */
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
                    } doe;
                } john;
                char *prop_name;
            };
        """

        expected_parameterlist = [
            '{unnamed_struct}',
            'a',
            'b',
            'c',
            'name',
            'auto_seq',
            'length1',
            'length2',
            'length3',
            'is_inline',

            'baz',
            'baz.undescribed_arg',
            'baz.bol',

            'fafafa',
            'fafafa.bar',
            'fafafa.{unnamed_struct}',

            'fafafa.foo2',
            'fafafa.foo2.{unnamed_struct}',
            'fafafa.foo2.my_enum',

            'fafafa.foo2.foo3',
            'fafafa.foo2.foo3.test',
            'fafafa.foo2.foo3.ano',

            'type',
            'literal',

            'john',
            'john.pointer',

            'john.doe',
            'john.doe.boou8_data',
            'john.doe.u16_data',
            'john.doe.u32_data',
            'john.doe.u64_data',
            'john.doe.str',

            'prop_name'
        ]

        self.parser = CDataParser(source)
        self.assertEqual(self.parser.item.parameterlist, expected_parameterlist)

    def test_func_params(self):
        source = """
            struct drm_pagemap_devmem_ops {

            void (*devmem_release)(struct drm_pagemap_devmem *devmem_allocation);

            int (*populate_devmem_pfn)(struct drm_pagemap_devmem *devmem_allocation,
                                       unsigned long npages, unsigned long *pfn);

            int (*copy_to_devmem)(struct page **pages,
                                  struct drm_pagemap_addr *pagemap_addr,
                                  unsigned long npages,
                                  struct dma_fence *pre_migrate_fence);
        };
        """

        expected_parameterlist = [
            "devmem_release",
            "populate_devmem_pfn",
            "copy_to_devmem",
       ]

        self.parser = CDataParser(source)
        self.assertEqual(self.parser.item.parameterlist, expected_parameterlist)


#
# Run all tests
#
if __name__ == "__main__":
    run_unittest(__file__)
