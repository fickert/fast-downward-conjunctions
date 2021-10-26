# enable minimal plugins to run LAMA
ENABLE_LAMA_PLUGINS = ["-DPLUGIN_FF_HEURISTIC_ENABLED=True", "-DPLUGIN_LAZY_SEARCH_ENABLED=True", "-DPLUGIN_ITERATED_SEARCH_ENABLED=True", "-DPLUGIN_LANDMARKS_ENABLED=True", "-DUSE_LP=NO"]

# enable partial delete relaxation plugins
ENABLE_PARTIAL_DELETE_RELAXATION_PLUGINS = ["-DPLUGIN_CONJUNCTIONS_ENABLED=True", "-DPLUGIN_GREY_ENABLED=True", "-DPLUGIN_RED_BLACK_ENABLED=True"]

# the default builds are reduced to the relevant configurations for satisficing planning and partial delete relaxation heuristics, other heuristics may not build correctly
release = ["-DCMAKE_BUILD_TYPE=Release", "-DDISABLE_PLUGINS_BY_DEFAULT=YES"] + ENABLE_LAMA_PLUGINS + ENABLE_PARTIAL_DELETE_RELAXATION_PLUGINS
debug = ["-DCMAKE_BUILD_TYPE=Debug", "-DDISABLE_PLUGINS_BY_DEFAULT=YES"] + ENABLE_LAMA_PLUGINS + ENABLE_PARTIAL_DELETE_RELAXATION_PLUGINS

release_clang = ['-DCMAKE_C_COMPILER=/usr/bin/clang', '-DCMAKE_CXX_COMPILER=/usr/bin/clang++'] + release
debug_clang = ['-DCMAKE_C_COMPILER=/usr/bin/clang', '-DCMAKE_CXX_COMPILER=/usr/bin/clang++'] + debug

DEFAULT = "release"
DEBUG = "debug"
