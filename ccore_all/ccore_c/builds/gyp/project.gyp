{
  'variables': {
    'lib%': 'static'
  },
  'includes': [
    '../../../builds/gyp/platform.gypi',
  ],
  'target_defaults': {
    'include_dirs': [
      '../../include',
      '.'
    ],
    'direct_dependent_settings': {
      'include_dirs': [
        '../../include'
      ]
    },
    'dependencies': [
      '../../../ccore_lib/builds/gyp/project.gyp:libccore',
      '../../../ccore_bson/builds/gyp/project.gyp:ccore_bson',
      '../../../libbson/builds/gyp/project.gyp:libbson'
    ],
    'export_dependent_settings': [
      '../../../ccore_lib/builds/gyp/project.gyp:libccore',
      '../../../ccore_bson/builds/gyp/project.gyp:ccore_bson',
      '../../../libbson/builds/gyp/project.gyp:libbson'
    ],
    'defines': [
      'CCORE_C_GYP_BUILD'
    ],
    'conditions': [
      [ 'OS=="win"', {
        'defines': [
          'CCORE_C_HAVE_WINDOWS',
          'ZMQ_STATIC',
          'CZMQ_STATIC',
          'BSON_STATIC',
          'CCORE_STATIC'
        ],
        'libraries': [
          'ws2_32',
          'advapi32',
          'iphlpapi',
          'Rpcrt4'
        ]
      }],
      [ 'OS=="mac"', {
        'defines': [
          'CCORE_C_HAVE_OSX'
        ],
        'xcode_settings': {
          'CLANG_CXX_LIBRARY': 'libc++',
          'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
          'MACOSX_DEPLOYMENT_TARGET':'10.11',
        },
        'cflags': [ '-std=c99' ],
      }],
      [ 'OS=="linux"', {
        'defines': [
          'CCORE_C_HAVE_LINUX'
        ],
        'cflags': [ '-std=c99' ],
        'libraries': [
          '-lpthread'
        ]
      }]
    ]
  },
  'targets': [
    {
      'target_name': 'ccore_c',
      'conditions': [
        [ 'lib=="static"', {
          'type': 'static_library',
          'defines': [
            'CCORE_STATIC',
            'ZMQ_STATIC',
            'CZMQ_STATIC',
          ]
        }],
        [ 'lib=="shared"', {
          'type': 'shared_library'
        }],
      ],
      'sources': [
        '../../include/ccore_c99.h',
        '../../include/ccore_poll_c99.h',
        '../../include/ccore_msg_c99.h',
        '../../src/ccore_c99.c',
        '../../src/ccore_poll_c99.c',
        '../../src/ccore_msg_c99.c',
      ],
    },
    {
      'target_name': 'ccore_c_basic_test',
      'type': 'executable',
      'sources': [
        '../../src/test/basic.c'
      ],
      'conditions': [
        [ 'lib=="static"', {
          'defines': [
            'CCORE_STATIC',
            'ZMQ_STATIC',
            'CZMQ_STATIC',
            'BSON_STATIC'
          ],
        }]
      ],
      'dependencies': [
        'ccore_c'
      ],
    },
    {
      'target_name': 'ccore_c_basic_poll_test',
      'type': 'executable',
      'sources': [
        '../../src/test/basic_poll.c'
      ],
      'conditions': [
        [ 'lib=="static"', {
          'defines': [
            'CCORE_STATIC',
            'ZMQ_STATIC',
            'CZMQ_STATIC',
            'BSON_STATIC'
          ],
        }]
      ],
      'dependencies': [
        'ccore_c'
      ],
    },
    {
      'target_name': 'copy-headers',
      'type': 'none',
        'copies': [{
          'destination': '<(PRODUCT_DIR)/include',
          'files': [
            '../../include/ccore_c99.h',
            '../../include/ccore_msg_c99.h',
            '../../include/ccore_poll_c99.h',
            '../../../ccore_lib/include/ccore_common.h',
            '../../../ccore_lib/include/ccore.h',
            '../../../ccore_lib/include/ccore_poll.h',
            '../../../ccore_bson/ccore_bson/include/bson.h',
            '../../../ccore_bson/ccore_bson/include/document.h',
            '../../../ccore_bson/ccore_bson/include/element.h',
            '../../../ccore_bson/ccore_bson/include/serializable.h',
            '../../../ccore_bson/ccore_bson/include/shared_buffer.h',
            '../../../ccore_bson/ccore_bson/include/typelist.h',
           ]
        }]
    },
    {
      'target_name': 'copy-bson-headers',
      'type': 'none',
        'copies': [{
          'destination': '<(PRODUCT_DIR)/include/bson',
          'files': [
            '../../../libbson/src/bson/b64_ntop.h',
            '../../../libbson/src/bson/b64_pton.h',
            '../../../libbson/src/bson/bcon.h',
            '../../../libbson/src/bson/bson-atomic.h',
            '../../../libbson/src/bson/bson-clock.h',
            '../../../libbson/src/bson/bson-compat.h',
            '../../../libbson/src/bson/bson-context-private.h',
            '../../../libbson/src/bson/bson-context.h',
            '../../../libbson/src/bson/bson-decimal128.h',
            '../../../libbson/src/bson/bson-endian.h',
            '../../../libbson/src/bson/bson-error.h',
            '../../../libbson/src/bson/bson-iso8601-private.h',
            '../../../libbson/src/bson/bson-iter.h',
            '../../../libbson/src/bson/bson-json.h',
            '../../../libbson/src/bson/bson-keys.h',
            '../../../libbson/src/bson/bson-macros.h',
            '../../../libbson/src/bson/bson-md5.h',
            '../../../libbson/src/bson/bson-memory.h',
            '../../../libbson/src/bson/bson-oid.h',
            '../../../libbson/src/bson/bson-private.h',
            '../../../libbson/src/bson/bson-reader.h',
            '../../../libbson/src/bson/bson-stdint-win32.h',
            '../../../libbson/src/bson/bson-string.h',
            '../../../libbson/src/bson/bson-thread-private.h',
            '../../../libbson/src/bson/bson-timegm-private.h',
            '../../../libbson/src/bson/bson-types.h',
            '../../../libbson/src/bson/bson-utf8.h',
            '../../../libbson/src/bson/bson-value.h',
            '../../../libbson/src/bson/bson-version-functions.h',
            '../../../libbson/src/bson/bson-writer.h',
            '../../../libbson/src/bson/bson.h',
            '../../../libbson/builds/gyp/<(OS)/bson-config.h',
            '../../../libbson/builds/gyp/<(OS)/bson-stdint.h',
            '../../../libbson/builds/gyp/<(OS)/bson-version.h'
           ],
      }]
    }
  ]
}
