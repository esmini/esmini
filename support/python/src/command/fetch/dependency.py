""" dependency.py handles the all the entities in the project that are classified as dependency

"""

from support.python.src.command.fetch.utils import (
    fetch_from_google_drive,
    replace_from_google_drive,
)
from support.python.src.globals import ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS
from support.python.src.utils import print_commands


class Dependency:
    """dependency command 'namespace'"""

    @staticmethod
    def fetch_dependency(key, args):
        """fetches the requested dependency from desired source"""

        Dependency._resolve_dependency_args(args)
        if args["name"] == "all" or args["name"][0] == "all":
            for dep in ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS:
                fetch_from_google_drive(key, dep, "dependency")
        else:
            for dep_name in args["name"]:
                for dep in ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS:
                    if dep[1] == dep_name:
                        fetch_from_google_drive(key, dep, "dependency")

    @staticmethod
    def replace_dependency(args):
        """replaces the requested dependency with desired methods"""

        version = ""
        Dependency._resolve_dependency_args(args)
        if args["name"] == "all" or args["name"][0] == "all":
            for dep in ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS:
                replace_from_google_drive(dep, "dependency", version)
        else:
            for dep_name in args["name"]:
                for dep in ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS:
                    if dep[1] == dep_name:
                        replace_from_google_drive(dep, "dependency", version)

    @staticmethod
    def _resolve_dependency_args(args):
        """resolves the dependency arguments in case there is conflicted request"""

        if (args["source"] != "google_drive") and (args["source"][0] != "google_drive"):
            raise ValueError("NOT SUPPORTED SOURCE", args["source"])

        if args["name"] != "all" and args["name"][0] != "all":
            match = False
            if isinstance(args["name"], str):
                args["name"] = [args["name"]]

            for desired_dep in args["name"]:
                for dep in ESMINI_GOOGLE_DRIVE_DEPENDENCY_COMPONENTS:
                    for dep_name in dep:
                        if dep_name == desired_dep:
                            match = dep_name
                if not match:
                    raise ValueError(
                        "DEPENDENCY NOT AVAILABLE",
                        "SEE HELP SECTION VIA",
                        "python3.8 start.py fetch dependency --help",
                    )

        print_commands(args)
