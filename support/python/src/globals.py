# pylint: skip-file
""" globals.py stores all needed variables that should be available globally

"""

import os

######################################################### DIRECTORIES ##############################################################

ESMINI_DIRECTORY_SCRIPT = os.path.dirname(os.path.realpath(__file__))
ESMINI_DIRECTORY_ROOT = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_SCRIPT, "..", "..", "..")
)
ESMINI_DIRECTORY_EXTERNALS = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "externals")
)
ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "EnvironmentSimulator")
)
ESMINI_DIRECTORY_SUPPORT = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "support")
)
ESMINI_DIRECTORY_RESOURCES = os.path.abspath(
    os.path.join(ESMINI_DIRECTORY_ROOT, "resources")
)

######################################################## EXTERNALS #################################################################

ESMINI_DEPENDENCY_EXTERNALS = {
    "google-drive": {
        "linux": {
            "osg": {
                "src": "https://drive.google.com/u/1/uc?id=1Ya1bLp_0-qqlhs67WAwbGW7l37wqP3o2&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osg"),
                "extension": ".7z"
            },
            "osi": {
                "src": "https://drive.google.com/u/1/uc?id=1Q8O9YciIC0BPEszIKtQ2UW9KcVRZS4iB&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osi"),
                "extension": ".7z"
            },
            "sumo": {
                "src": "https://drive.google.com/u/1/uc?id=1m4znxNIXapP0D-l21oIm2l7L5ti-JbZH&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "sumo"),
                "extension": ".7z"
            },
            "googletest": {
                "src": "https://drive.google.com/u/1/uc?id=1Hyr9eJX2GmgpYwZhx14xOoXlZ2j-FY_p&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "googletest"),
                "extension": ".7z"
            },
            "models": {
                "src": "https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_RESOURCES),
                "extension": ".7z"
            }
        },
        "mac": {
            "osg": {
                "src": "https://drive.google.com/u/1/uc?id=1mfn_vrcXBoFBekR_t8RXTWB4sD59JD7p&export=download ",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osg"),
                "extension": ".7z"
            },
            "osi": {
                "src": "https://drive.google.com/u/1/uc?id=1UVzO8cPQaDU9KVn9v2v8Suj0uUw1dzYI&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osi"),
                "extension": ".7z"
            },
            "sumo": {
                "src": "https://drive.google.com/u/1/uc?id=1FAve0-MlJPv6lUZy0HvriZI7xstLAzvX&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "sumo"),
                "extension": ".7z"
            },
            "googletest": {
                "src": None,
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "googletest"),
                "extension": ".7z"
            },
            "models": {
                "src": "https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_RESOURCES, "models"),
                "extension": ".7z"
            }
        },
        "msvc": {
            "osg": {
                "src": "https://drive.google.com/u/1/uc?id=1RTag0aUn_pJPK697j0-E72ABW10wZvOm&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osg"),
                "extension": ".7z"
            },
            "osi": {
                "src": "https://drive.google.com/u/1/uc?id=1pcQcVHUESOk2Wmi-zUA7uzdxxE6iwRJx&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "osi"),
                "extension": ".7z"
            },
            "sumo": {
                "src": "https://drive.google.com/u/1/uc?id=18PhbSLyvs0IGWTAY3YBoYzpVnMFPbOuR&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "sumo"),
                "extension": ".7z"
            },
            "googletest": {
                "src": "https://drive.google.com/u/1/uc?id=1So-3gtrmEdW9RhEvVQisj1QFksHM_otU&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_EXTERNALS, "googletest"),
                "extension": ".7z"
            },
            "models": {
                "src": "https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download",
                "destination": os.path.join(ESMINI_DIRECTORY_RESOURCES, "models"),
                "extension": ".7z"
            }
        }
    }
}

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
    os.path.join(ESMINI_DIRECTORY_ROOT, "CMakeLists.txt"),
    os.path.join(ESMINI_DIRECTORY_SUPPORT, "cmake"),
    os.path.join(ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR, "Modules"),
    os.path.join(ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR, "Libraries"),
    os.path.join(ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR, "Applications", "esmini"),
    os.path.join(ESMINI_DIRECTORY_ENVIRONMENT_SIMULATOR, "Applications", "esmini-dyn"),
]

ESMINI_CMAKE_FORMAT_EXTENSION = ["CMakeLists.txt", ".cmake"]

ESMINI_CLANG_FORMAT_INCLUDES = [
]
ESMINI_CLANG_FORMAT_EXCLUDES = []
ESMINI_CLANG_FORMAT_EXTENSION = [".cpp", ".c", ".hpp", ".h"]

########################################################## OTHERS ############################################################

SEPARATOR = 80
