import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import xmlschema
from lxml import etree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_schema_comply import XmlValidation

SCHEMA_DIR = Path(__file__).resolve().parents[1] / "resources" / "schema"
SOURCE = SCHEMA_DIR / "OpenSCENARIOv1.4.xsd"
OUTPUT = SCHEMA_DIR / "OpenSCENARIOv1.4-esmini.xsd"


class EsminiSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.standard = xmlschema.XMLSchema(SOURCE)
        cls.extended = xmlschema.XMLSchema11(OUTPUT)
        cls.lxml_standard = etree.XMLSchema(etree.parse(str(SOURCE)))

    def test_wrapper_only_overrides_parameter_and_expression(self):
        root = etree.parse(str(OUTPUT)).getroot()
        ns = {"xsd": "http://www.w3.org/2001/XMLSchema"}
        self.assertEqual(
            root.get("{http://www.w3.org/2007/XMLSchema-versioning}minVersion"), "1.1"
        )
        self.assertEqual(
            [etree.QName(child).localname for child in root],
            ["annotation", "override"],
        )
        override = root.find("xsd:override", ns)
        self.assertEqual(override.get("schemaLocation"), SOURCE.name)
        self.assertEqual(
            [(etree.QName(child).localname, child.get("name")) for child in override],
            [("simpleType", "parameter"), ("simpleType", "expression")],
        )

    def test_numeric_attribute_boundary(self):
        cases = [
            ("2.4", True, True),
            ("$Width", True, True),
            ("${$Width / 2}", True, True),
            ("$CarWhite::Width", False, True),
            ("${$CarWhite::Width / 2 + 0.5}", False, True),
            ("${$CarWhite::Width + $CarBlue::Width}", False, True),
            ("$CarWhite:Width", False, False),
            ("${$CarWhite:Width / 2}", False, False),
            ("${$CarWhite:::Width / 2}", False, False),
            ("$CarWhite::", False, False),
            ("$CarWhite::Width::Extra", False, False),
            ("$1Car::Width", False, False),
        ]
        for value, standard, extended in cases:
            with self.subTest(value=value):
                document = etree.Element("OpenSCENARIO")
                etree.SubElement(
                    document,
                    "FileHeader",
                    revMajor="1",
                    revMinor="4",
                    date="2026-10-21T00:00:00",
                    description="Schema test",
                    author="esmini",
                )
                etree.SubElement(document, "ParameterDeclarations")
                etree.SubElement(document, "CatalogLocations")
                etree.SubElement(document, "RoadNetwork")
                etree.SubElement(document, "Entities")
                storyboard = etree.SubElement(document, "Storyboard")
                etree.SubElement(etree.SubElement(storyboard, "Init"), "Actions")
                stop = etree.SubElement(storyboard, "StopTrigger")
                group = etree.SubElement(stop, "ConditionGroup")
                condition = etree.SubElement(
                    group, "Condition", name="Stop", delay="0", conditionEdge="none"
                )
                by_value = etree.SubElement(condition, "ByValueCondition")
                etree.SubElement(
                    by_value, "SimulationTimeCondition", value=value, rule="greaterThan"
                )
                self.assertEqual(self.standard.is_valid(document), standard)
                self.assertEqual(self.extended.is_valid(document), extended)
                self.assertEqual(self.lxml_standard.validate(document), standard)

    def test_default_schema_selection_is_version_specific(self):
        standard = XmlValidation()
        self.assertEqual(
            Path(standard.get_xsd_to_validate("4", "test.xosc")).resolve(), SOURCE
        )
        for revision, filename, schema in (
            ("3", "test.xosc", "OpenSCENARIOv1.3.xsd"),
            ("4", "test.xodr", "OpenDRIVE_1.4H.xsd"),
        ):
            self.assertEqual(
                Path(standard.get_xsd_to_validate(revision, filename)).resolve(),
                SCHEMA_DIR / schema,
            )

    def test_validator_class_matches_xsd_version(self):
        validator = XmlValidation()
        self.assertIsInstance(
            validator.get_scheme("test.xosc", str(SOURCE)), xmlschema.XMLSchema10
        )
        self.assertIsInstance(
            validator.get_scheme("test.xosc", str(OUTPUT)), xmlschema.XMLSchema11
        )
        self.assertIsInstance(
            validator.get_scheme(
                "test.xodr",
                str(SCHEMA_DIR / "OpenDRIVE_1.8" / "local_schema" / "OpenDRIVE_Core.xsd"),
            ),
            xmlschema.XMLSchema11,
        )
        self.assertEqual(validator.count_of_files_failed, 0)

    def test_cli_follows_schema_hint_and_preserves_fallback(self):
        document = """<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:noNamespaceSchemaLocation="OpenSCENARIOv1.4-esmini.xsd">
            <FileHeader revMajor="1" revMinor="4" date="2026-10-21T00:00:00"
                description="Schema test" author="esmini"/>
            <CatalogLocations/><RoadNetwork/><Entities/>
            <Storyboard><Init><Actions/></Init><StopTrigger><ConditionGroup>
                <Condition name="Stop" delay="0" conditionEdge="none"><ByValueCondition>
                    <SimulationTimeCondition value="${$Car1::Width / 2}" rule="greaterThan"/>
                </ByValueCondition></Condition>
            </ConditionGroup></StopTrigger></Storyboard>
        </OpenSCENARIO>"""
        script = SOURCE.parents[2] / "scripts" / "run_schema_comply.py"
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "qualified.xosc"
            for hint, expected in (
                (OUTPUT.as_uri(), 0),
                (SOURCE.as_uri(), 1),
                ("missing.xsd", 1),
                (None, 1),
            ):
                with self.subTest(hint=hint):
                    root = etree.fromstring(document.encode())
                    key = "{http://www.w3.org/2001/XMLSchema-instance}noNamespaceSchemaLocation"
                    if hint is None:
                        del root.attrib[key]
                    else:
                        root.set(key, hint)
                    scenario.write_bytes(etree.tostring(root))
                    result = subprocess.run(
                        [sys.executable, str(script), str(scenario)],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    self.assertEqual(
                        result.returncode, expected, result.stdout + result.stderr
                    )
                    if hint == "missing.xsd":
                        self.assertIn("[WARNING]", result.stdout)
                        self.assertIn("Falling back", result.stdout)
                    if hint in ("missing.xsd", None):
                        root.find(".//SimulationTimeCondition").set("value", "2.4")
                        scenario.write_bytes(etree.tostring(root))
                        result = subprocess.run(
                            [sys.executable, str(script), str(scenario)],
                            capture_output=True, text=True, check=False,
                        )
                        self.assertEqual(
                            result.returncode, 0, result.stdout + result.stderr
                        )

    def test_missing_schema_fallback_uses_file_version(self):
        with tempfile.TemporaryDirectory() as directory:
            for revision, expected in (("3", 1), ("4", 1), ("99", 0)):
                with self.subTest(revision=revision):
                    scenario = Path(directory) / "example.xosc"
                    scenario.write_text(
                        '<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                        'xsi:noNamespaceSchemaLocation="OpenSCENARIO.xsd">'
                        f'<FileHeader revMajor="1" revMinor="{revision}" '
                        'date="2026-10-21T00:00:00" description="Test" author="esmini"/>'
                        '<CatalogLocations/><RoadNetwork/><Entities/>'
                        '<Storyboard><Init><Actions/></Init><StopTrigger/></Storyboard>'
                        '</OpenSCENARIO>',
                        encoding="utf-8",
                    )
                    validator = XmlValidation()
                    output = StringIO()
                    with redirect_stdout(output):
                        validator.main([str(scenario)])
                    self.assertIn("[WARNING]", output.getvalue())
                    self.assertEqual(validator.count_of_files_validated, expected)
                    self.assertEqual(validator.count_of_files_failed, 1 - expected)

    def test_local_schema_resolution_and_errors(self):
        key = "{http://www.w3.org/2001/XMLSchema-instance}noNamespaceSchemaLocation"
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            schemas = base / "schemas"
            schemas.mkdir()
            schema = schemas / "custom schema.xsd"
            schema.write_text(
                '<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema">'
                '<xs:element name="example" type="xs:string"/></xs:schema>',
                encoding="utf-8",
            )
            scenario = base / "example.xosc"
            for hint in ("schemas/custom schema.xsd", str(schema), schema.as_uri()):
                with self.subTest(hint=hint):
                    root = etree.Element(
                        "example",
                        nsmap={"other": "http://www.w3.org/2001/XMLSchema-instance"},
                    )
                    root.set(key, hint)
                    scenario.write_bytes(etree.tostring(root))
                    validator = XmlValidation()
                    validator.main([str(scenario)])
                    self.assertEqual(validator.count_of_files_validated, 1)
                    self.assertEqual(validator.count_of_files_failed, 0)
            for hint in ("", "https://example.invalid/schema.xsd"):
                with self.subTest(hint=hint):
                    root.set(key, hint)
                    scenario.write_bytes(etree.tostring(root))
                    validator = XmlValidation()
                    validator.main([str(scenario)])
                    self.assertEqual(validator.count_of_files_validated, 0)
                    self.assertEqual(validator.count_of_files_failed, 1)
                    self.assertTrue(validator.errors)

    def test_silent_cli_suppresses_warnings_but_preserves_results_and_errors(self):
        script = SOURCE.parents[2] / "scripts" / "run_schema_comply.py"
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "example.xosc"
            for revision, expected, marker in (("4", 0, "[OK]"), ("99", 1, "[NOT OK]")):
                with self.subTest(revision=revision):
                    scenario.write_text(
                        '<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                        'xsi:noNamespaceSchemaLocation="OpenSCENARIO.xsd">'
                        f'<FileHeader revMajor="1" revMinor="{revision}" '
                        'date="2026-10-21T00:00:00" description="Test" author="esmini"/>'
                        '<CatalogLocations/><RoadNetwork/><Entities/>'
                        '<Storyboard><Init><Actions/></Init><StopTrigger/></Storyboard>'
                        '</OpenSCENARIO>',
                        encoding="utf-8",
                    )
                    result = subprocess.run(
                        [sys.executable, str(script), "--silent", str(scenario)],
                        capture_output=True, text=True, check=False,
                    )
                    self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
                    self.assertNotIn("[WARNING]", result.stdout)
                    self.assertIn(marker, result.stdout)
                    self.assertIn("files validated", result.stdout)

    def test_invalid_schema_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            schema = Path(directory) / "broken.xsd"
            for content in (
                "not XML",
                '<xsd:schema xmlns:xsd="http://www.w3.org/2001/XMLSchema"><xsd:unknown/></xsd:schema>',
            ):
                with self.subTest(content=content):
                    schema.write_text(content, encoding="utf-8")
                    validator = XmlValidation()
                    self.assertIsNone(validator.get_scheme("example.xosc", str(schema)))
                    self.assertEqual(validator.count_of_files_failed, 1)

    def test_fail_fast_stops_on_first_error_and_default_continues(self):
        script = SOURCE.parents[2] / "scripts" / "run_schema_comply.py"
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            schema = base / "example.xsd"
            schema.write_text(
                '<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema">'
                '<xs:element name="example"><xs:complexType>'
                '<xs:attribute name="first" type="xs:int" use="required"/>'
                '<xs:attribute name="second" type="xs:int" use="required"/>'
                '</xs:complexType></xs:element></xs:schema>',
                encoding="utf-8",
            )
            valid = (
                '<example xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                'xsi:noNamespaceSchemaLocation="example.xsd" first="1" second="2"/>'
            )
            before = base / "before.xosc"
            failed = base / "failed.xosc"
            after = base / "after.xosc"
            before.write_text(valid, encoding="utf-8")
            after.write_text(valid, encoding="utf-8")
            for content in (
                valid.replace('first="1" second="2"', 'first="bad" second="bad"'),
                "not XML",
                "<OpenSCENARIO/>",
                '<OpenSCENARIO><FileHeader revMinor="99"/></OpenSCENARIO>',
                valid.replace("example.xsd", "broken.xsd"),
            ):
                (base / "broken.xsd").write_text("not a schema", encoding="utf-8")
                failed.write_text(content, encoding="utf-8")
                for fail_fast in (False, True):
                    with self.subTest(content=content, fail_fast=fail_fast):
                        validator = XmlValidation(fail_fast=fail_fast)
                        with redirect_stdout(StringIO()):
                            validator.main([str(before), str(failed), str(after)])
                        self.assertEqual(validator.count_of_files_failed, 1)
                        self.assertEqual(
                            validator.count_of_files_validated, 1 if fail_fast else 2
                        )
                        if fail_fast:
                            self.assertEqual(len(validator.errors), 1)
                        elif 'first="bad"' in content:
                            self.assertEqual(len(validator.errors), 2)
                result = subprocess.run(
                    [sys.executable, str(script), "--fail-fast",
                     str(before), str(failed), str(after)],
                    capture_output=True, text=True, check=False,
                )
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("[NOT OK]", result.stdout)
                self.assertNotIn(str(after), result.stdout)


if __name__ == "__main__":
    unittest.main()
