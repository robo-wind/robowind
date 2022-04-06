{
  'includes': [
    '../../../../builds/gyp/platform.gypi',
  ],
  'target_defaults': {
    'include_dirs': [
      '../../include',
      '.'
    ],
    'dependencies': [
      '../../../builds/gyp/project.gyp:ccore_cpp',
    ],
    'defines': [
      'CCORE_CPP_SAMPLE_GYP_BUILD'
    ],
    'conditions': [
      [ 'OS=="win"', {
        'defines': [
          'CCORE_SAMPLE_HAVE_WINDOWS',
          'ZMQ_STATIC',
          'CZMQ_STATIC',
          'BSON_STATIC',
          'CCORE_STATIC',
          'CCORE_CPP_STATIC'
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
          'CCORE_CPP_SAMPLE_HAVE_OSX'
        ],
        'xcode_settings': {
          'CLANG_CXX_LIBRARY': 'libc++',
          'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
          'MACOSX_DEPLOYMENT_TARGET':'10.11',
        },
      }],
      [ 'OS=="linux"', {
        'defines': [
          'CCORE_CPP_SAMPLE_HAVE_LINUX'
        ],
        'libraries': [
          '-lpthread'
        ]
      }]
    ]
  },
  'targets': [
    {
      'target_name': 'ccore_libbson_example',
      'type': 'executable',
      'sources': [
        '../../src/libbson_example.cpp'
      ],
    },
    {
      'target_name': 'ccore_example',
      'type': 'executable',
      'sources': [
        '../../src/ccore_example.cpp'
      ],
    },
  ]
}
