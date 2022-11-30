# pylint: skip-file
""" globals.py stores all needed variables that should be available globally

"""

import os

######################################################### DIRECTORIES ##############################################################

ESMINI_DIRECTORY_SCRIPT = os.path.dirname(os.path.realpath(__file__))
ESMINI_DIRECTORY_ROOT = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_SCRIPT, "..", "..", "..")
)
ESMINI_DIRECTORY_EXTERNAL = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "externals")
)
ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "EnvironmentSimulator")
)
ESMINI_DIRECTORY_SUPPORT = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "support")
)

######################################################## GOOGLE_DRIVE LINKS ########################################################

ESMINI_GOOGLE_DRIVE_DEPENDENCY_EXTERNAL = (
    "https://ara-artifactory.volvocars.biz/artifactory/ESMINI-lts/externals"
)

######################################################## GOOGLE_DRIVE COMPONENTS ###################################################

ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS = [
    [ESMINI_GOOGLE_DRIVE_DEPENDENCY_EXTERNAL, "osi", "X.X.X"],
    [ESMINI_GOOGLE_DRIVE_DEPENDENCY_EXTERNAL, "open_scene_graph", "X.X.X"],
    [ESMINI_GOOGLE_DRIVE_DEPENDENCY_EXTERNAL, "sumo", "X.X.X"],
    [ESMINI_GOOGLE_DRIVE_DEPENDENCY_EXTERNAL, "googletest", "X.X.X"],
]

####################################################### CMAKE TARGETS ##############################################################

ESMINI_CMAKE_TARGET_FLAGS = [
    "USE_OSG=true",
    "USE_OSI=true",
    "USE_SUMO=true",
    "USE_GTEST=true",
    "DYN_PROTOBUF=false",
    "ENABLE_SANITIZERS=false",
]

####################################################### COMPILERS ##################################################################

ESMINI_COMPILERS = [
    "gcc-7",
    "gcc-9",
    "gcc-11",
    "clang-10",
    "clang-12",
    "clang-15",
    "clang-16",
]

########################################################## FORMAT ############################################################

ESMINI_CMAKE_FORMAT_INCLUDES = [
]

ESMINI_CMAKE_FORMAT_EXTENSION = ["CMakeLists.txt", ".cmake"]

ESMINI_CLANG_FORMAT_INCLUDES = [
]
ESMINI_CLANG_FORMAT_EXCLUDES = []
ESMINI_CLANG_FORMAT_EXTENSION = [".cpp", ".c", ".hpp", ".h"]

########################################################## OTHERS ############################################################

SEPARATOR = 80
