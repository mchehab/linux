#!/usr/bin/env python3

"""
Unit tests for struct/union member extractor class.
"""


import os
import unittest
import sys

from unittest.mock import MagicMock
from textwrap import dedent


SRC_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(SRC_DIR, "../lib/python"))

from kdoc.kdoc_re import MemberExtractor
from unittest_helper import run_unittest

#
# List of tests.
#
# The code will dynamically generate one test for each key on this dictionary.
#
TESTS = {
    #
    # Invalid data
    #
    "no_struct": {
        "source": "int foo;",
        "id": None,
        "members": [],
    },

    "no_struct_braces": {
        "source": "struct foo;",
        "id": "foo",
        "members": [],
    },

    #
    # Start with basics: very simple struct with no inner struct/union
    #
    "simple_struct": {
        "source": "struct foo { int a; };",
        "id": "foo",
        "members": [
            ["a"],
        ],
    },

    #
    # check simple inner anonymous struct: members lists have just one element
    #
    "simple_inner_anonymous_struct": {
        "source": dedent("""
            struct outer {
                int x;
                struct inner { float y; };
            };
        """),
        "id": "outer",
        "members": [
            ["x"],
            ["y"],
        ],
    },

    #
    # simple inner named struct: detect if members level will be filled
    #
    "simple_inner_named_struct": {
        "source": dedent("""
            struct outer2 {
                int x;
                struct inner { float y; } z;
            };
        """),
        "id": "outer2",
        "members": [
            ["x"],
            ["z", "y"],
        ],
    },

    #
    # Check if decoding a pointer to an array will work
    #
    "array_and_pointer": {
        "source": dedent("""
            struct bar {
                int *ptr;
                char *arr[10];
            };
        """),
        "id": "bar",
        "members": [
            ["ptr"],
            ["arr"],
        ],
    },

    #
    # complex case with a mix of inner structs and unions
    #
    "complex_struct": {
        # Derivated from include/linux/property.h, line 397
        "source": dedent("""
            struct property_entry {
                const char *name;
                size_t length;
                bool is_inline;
                enum dev_prop_type type;
                union {
                    const void *pointer;
                    struct {
                        u8 u8_data[sizeof(u64) / sizeof(u8)];
                        u16 u16_data[sizeof(u64) / sizeof(u16)];
                        u32 u32_data[sizeof(u64) / sizeof(u32)];
                        u64 u64_data[sizeof(u64) / sizeof(u64)];
                        const char *str[sizeof(u64) / sizeof(char *)];
                    } value;
                };
                struct {
                    const void *pointer;
                    union {
                        u8 u8_data[sizeof(u64) / sizeof(u8)];
                        u16 u16_data[sizeof(u64) / sizeof(u16)];
                        u32 u32_data[sizeof(u64) / sizeof(u32)];
                        u64 u64_data[sizeof(u64) / sizeof(u64)];
                        const char *str[sizeof(u64) / sizeof(char *)];
                    } value2;
                } foo;
            }
        """),
        "id": "property_entry",
        "members": [
            #
            # Direct members
            #
            ["name"],
            ["length"],
            ["is_inline"],
            ["type"],
            ["pointer"],

            #
            # Members from a named struct inside an anonymous union
            #
            ["value", "u8_data"],
            ["value", "u16_data"],
            ["value", "u32_data"],
            ["value", "u64_data"],
            ["value", "str"],

            #
            # 3-level Members from both named struct and union
            #
            ['foo', 'pointer'],
            ["foo", "value2", "u8_data"],
            ["foo", "value2", "u16_data"],
            ["foo", "value2", "u32_data"],
            ["foo", "value2", "u64_data"],
            ["foo", "value2", "str"],
        ],
    },

    "struct_group_tagged": {
        # Derivated from include/net/page_pool/types.h line 78
        "source": dedent("""
                struct page_pool_params {
                    struct page_pool_params_fast {
                        unsigned int order;
                        unsigned int pool_size;
                        int nid;
                        struct device *dev;
                        struct napi_struct *napi;
                        enum dma_data_direction dma_dir;
                        unsigned int max_len;
                        unsigned int offset;
                    } fast;
                    struct page_pool_params_slow {
                        struct net_device *netdev;
                        unsigned int queue_idx;
                        unsigned int flags;
                    } slow;
                };
        """),
        "id": "page_pool_params",
        "members": [
            ['fast', 'order'],
            ['fast', 'pool_size'],
            ['fast', 'nid'],
            ['fast', 'dev'],
            ['fast', 'napi'],
            ['fast', 'dma_dir'],
            ['fast', 'max_len'],
            ['fast', 'offset'],
            ['slow', 'netdev'],
            ['slow', 'queue_idx'],
            ['slow', 'flags'],
        ],
    },
}


class TestMemberExtractor(unittest.TestCase):
    """
    Main test class. Populated dynamically at runtime.
    """

    def setUp(self):
        self.maxDiff = None

    def add_test(cls, name, expected_id, source, members):
        """
        Dynamically add a test to the class
        """
        def test(cls):
            logger = MagicMock()

            extractor = MemberExtractor(logger)
            s_id, result = extractor.parse(source)

            cls.assertEqual(s_id, expected_id, msg=f'failed on {name}')

            cls.assertEqual(result, members, msg=f'failed on {name}')

        test.__name__ = f'test_{name}'

        setattr(TestMemberExtractor, test.__name__, test)


#
# Populate TestMemberExtractor class
#
test_class = TestMemberExtractor()
for name, test in TESTS.items():
    test_class.add_test(name, test["id"], test["source"], test["members"])


#
# main
#
if __name__ == "__main__":
    run_unittest(__file__)
