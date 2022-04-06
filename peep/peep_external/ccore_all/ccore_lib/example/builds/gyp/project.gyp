{
  'includes': [
    '../../../../builds/gyp/platform.gypi',
  ],
  'target_defaults': {
    'include_dirs': [
      '../../include',
    ],
    'dependencies': [
      '../../../../ccore_lib/builds/gyp/project.gyp:libccore',
    ],
    'conditions': [
      [ 'OS=="win"', {
        'defines': [
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
        'xcode_settings': {
          'CLANG_CXX_LIBRARY': 'libc++',
          'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
          'MACOSX_DEPLOYMENT_TARGET':'10.11',
        },
      }],
      [ 'OS=="linux"', {
        'libraries': [
          '-lpthread'
        ]
      }]
    ]
  },
  'targets': [
    {
        'target_name': 'perf',
        'type': 'executable',
        'sources': [
            '../../src/perf.cpp'
        ],
        'include_dirs': [
            '../../third-party',
        ],
    },

    {
      'target_name': 'ccore_example',
      'type': 'executable',
      'sources': [
        '../../src/ccore_example.cpp'
      ],
    },
    {
      'target_name': 'ccore_example_object',
      'type': 'executable',
      'sources': [
        '../../src/ccore_example_object.cpp'
      ],
    },
    {
      'target_name': 'ccore_example_failover',
      'type': 'executable',
      'sources': [
        '../../src/ccore_example_failover.cpp'
      ],
    },
    {
      'target_name': 'ccore_poll_example_failover',
      'type': 'executable',
      'sources': [
        '../../src/ccore_poll_example_failover.cpp'
      ],
    },
    {
      'target_name': 'ccore_task_example',
      'type': 'executable',
      'sources': [
        '../../src/ccore_task_example.cpp'
      ],
    }
  ]
}
