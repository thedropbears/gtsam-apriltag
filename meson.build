project('gtsam_apriltag', ['cpp'],
        default_options: ['warning_level=1', 'cpp_std=c++23',
                          'b_colorout=auto', 'optimization=2', 'b_pie=true'])

subdir('semiwrap')

gtsam_apriltag_sources += files(
  'gtsam_apriltag/src/gtsam_apriltag.cpp',
  'gtsam_apriltag/src/main.cpp',
)

cpp = meson.get_compiler('cpp')
libs_path = meson.current_source_dir() + '/gtsam/lib'

gtsam_apriltag_deps += [
  cpp.find_library('libgtsamRelWithDebInfo', dirs: libs_path, static: true),
  declare_dependency(include_directories: [
    'gtsam_apriltag/include',
    'gtsam/include',
    ])
]

subdir('semiwrap/modules')