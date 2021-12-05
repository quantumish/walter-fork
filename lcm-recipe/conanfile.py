from conans import CMake, ConanFile, tools


class LcmConan(ConanFile):
    name = "lcm"
    version = "1.4.0"
    license = "LGPL-2.1"
    description = "Lightweight networking library."
    topics = "networking"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = "cmake"

    def config_options(self):
        pass

    def source(self):
        self.run("git clone https://github.com/lcm-proj/lcm.git")

    def build(self):
        cmake = CMake(self)
        cmake.definitions["LCM_ENABLE_JAVA"] = "OFF"
        cmake.definitions["LCM_ENABLE_GO"] = "OFF"
        cmake.definitions["LCM_ENABLE_LUA"] = "OFF"
        cmake.definitions["LCM_ENABLE_PYTHON"] = "OFF"
        self.run("cmake lcm %s" % cmake.command_line)
        self.run("make -C lcm")
        self.run("make -C lcm install")
