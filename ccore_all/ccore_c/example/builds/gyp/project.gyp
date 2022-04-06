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
      '../../../../ccore_c/builds/gyp/project.gyp:ccore_c',
    ],
    'defines': [
      'CCORE_C_SAMPLE_GYP_BUILD'
    ],
    'conditions': [
      [ 'OS=="win"', {
        'defines': [
          'CCORE_C_SAMPLE_HAVE_WINDOWS',
          'ZMQ_STATIC',
          'CZMQ_STATIC',
          'BSON_STATIC',
          'CCORE_C_STATIC'
        ],
        'libraries': [
          'ws2_32',
          'advapi32',
          'iphlpapi',
          'Rpcrt4',
          'c++'
        ]
      }],
      [ 'OS=="mac"', {
        'defines': [
          'CCORE_C_SAMPLE_HAVE_OSX'
        ],
        'xcode_settings': {
          'CLANG_CXX_LIBRARY': 'libc++',
          'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
          'MACOSX_DEPLOYMENT_TARGET':'10.11',
          'OTHER_LDFLAGS': ['-lc++']
        },
      }],
      [ 'OS=="linux"', {
        'defines': [
          'CCORE_C_SAMPLE_HAVE_LINUX'
        ],
        'libraries': [
          '-lpthread'
        ]
      }]
    ]
  },
  'targets': [
    {
      'target_name': 'ccore_c_example',
      'type': 'executable',
      'sources': [
        '../../src/ccore_example.c'
      ]
    },
    {
      'target_name': 'ccore_c_task_example',
      'type': 'executable',
      'sources': [
        '../../src/ccore_task_example.c'
      ]
    }
  ]
}
