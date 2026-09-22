project('estimator', ['cpp'],
        default_options: ['warning_level=1', 'cpp_std=c++23',
                          'b_colorout=auto', 'optimization=2', 'b_pie=true'])

subdir('semiwrap')

estimator_sources += files(
  'gtsam_apriltag/src/estimator.cpp',
  'gtsam_apriltag/src/main.cpp',
)

cpp = meson.get_compiler('cpp')
libs_path = meson.current_source_dir() + '/gtsam/lib'

estimator_deps += [
  cpp.find_library('libgtsamRelWithDebInfo', dirs: libs_path, static: true, required: true),
  cpp.find_library('libmetis-gtsamRelWithDebInfo', dirs: libs_path, static: true, required: true),
  cpp.find_library('libcephes-gtsamRelWithDebInfo', dirs: libs_path, static: true, required: true),
  declare_dependency(include_directories: [
    'gtsam_apriltag/include',
    'gtsam/include',
    ])
]

subdir('semiwrap/modules')