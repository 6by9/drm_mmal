project(
  'mmal-demo',
  ['c',],
  version : '1.0',
  license : 'MIT',
  default_options : [
    'buildtype=debugoptimized',
    'c_std=gnu99'
  ],
)
cc = meson.get_compiler('c')

opt_vc = get_option('opt_vc')

mmal_libs = [
  'vcsm',
  'mmal',
  'mmal_core',
  'mmal_util',
  'mmal_vc_client',
  'bcm_host',
  'vcos',
]

mmal_dep = []
foreach lib: mmal_libs
  mmal_dep += declare_dependency(
    link_args: [
      '-L@0@/lib'.format(opt_vc),
      '-l@0@'.format(lib),
      '-Wl,-rpath-link,@0@/lib'.format(opt_vc),
    ],
    include_directories: include_directories('@0@/include'.format(opt_vc))
  )
endforeach

executable(
  'mmal-demo',
  'drm_mmal.c',
  dependencies: [
    dependency('threads'),
    cc.find_library('rt'),
    dependency('libdrm'),
    mmal_dep,
  ],
)
