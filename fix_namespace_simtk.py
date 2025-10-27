#!/usr/bin/env python3
"""
Script to remove "using namespace SimTK;" and prefix SimTK types
Run it, compile, add missing types, run again until it works.
"""

import re
from pathlib import Path

# ========== CONFIG ==========
TARGET_DIR = "./"  # Opensim core dir

SIMTK_TYPES = [
    "Vec1",
    "Vec2",
    "Vec3",
    "Vec4",
    "Vec5",
    "Vec6",
    "Vec7",
    "Vec8",
    "Vec9",
    "Vec<10>",
    "Vec<11>",
    "Vec<12>",
    r'Vec<[0-9a-zA-Z]+>',
    "Vector",
    "Vector_",
    "RowVector",
    "RowVector_",
    "Matrix",
    "State",
    "System",
    "Transform",
    "Rotation",
    "Real",
    "Mat33",
    "SimbodyMatterSubsystem",
    "GeneralForceSubsystem",
    "BodyRotationSequence",
    "Force",
    "MobilizedBody",
    "MobilizedBodyIndex",
    "Body",
    "Inertia",
    "MultibodySystem",
    "Xml::Comment",
    "Xml::Element",
    "Xml::element_iterator",
    "SpatialVec",
    "UnitVec3",
    "Quaternion",
    "RungeKuttaMersonIntegrator",
    "RungeKuttaFeldbergIntegrator",
    "TimeStepper",
    "Stage::Dynamics",
    "Stage::Position",
    "Stage::Model",
    "Stage::Velocity",
    "Stage::Acceleration",
    "Stage::Report",
    "Stage::Time",
    "Stage",
    "Value",
    "ReferencePtr",
    "MassProperties",
    "ForceIndex",
    "Integrator",
    "VectorView",
    "Pi",
    "readUnformatted",
    "writeUnformatted",
    "[XYZ]Axis",
    "BodyOrSpaceType",
    "Infinity",
    "NaN",
    "VectorView_",
    "Subsystem",
    "AbstractMeasure",
    "Measure",
    "Measure_",
    "ConstrainedBodyIndex",
    "Random",
    "String",
    "Array_",
    "SubsystemIndex",
    "DiscreteVariableIndex",
    "DefaultSystemSubsystem",
    "FactorSVD",
    "Optimizer",
    "OptimizerAlgorithm",
    "Markers",
    "Assembler",
    "OrientationSensors",
    # Add more types here as you discover them
]

FILE_EXTENSIONS = [".cpp", ".hpp", ".h", ".cc", ".cxx"]
# ============================

FILES_TO_EXAMINE = ["AbstractProperty.cpp"]
FILES_TO_EXAMINE = ["testSerialization.cpp"]
FILES_TO_EXAMINE = [""]

def process_file(file_path):
    """Remove using namespace SimTK and prefix types."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    VERBOSE = False

    if file_path.parts[-1] in FILES_TO_EXAMINE:
        VERBOSE = True
    original_content = content
    
    # Remove "using namespace SimTK;" lines
    content = re.sub(r'^\s*using\s+namespace\s+SimTK\s*;\s*\n', '', content, flags=re.MULTILINE)
    
    if content == original_content:
        print(f"no SimTK namespace def in file {file_path}, skipping.")
        return
    
    # Prefix SimTK types
    for simtk_type in SIMTK_TYPES:
        # Match the type as a whole word, not preceded by :: or SimTK::
        # Also not followed by more word characters (to avoid InertiaTester, etc.)
        # Negative lookbehind: (?<!::) means "not preceded by ::"
        # Negative lookahead: (?![a-zA-Z0-9_]) means "not followed by word chars"
        ## hack
        trail_re = r'(?![a-zA-Z0-9_])'
        if "_" in simtk_type:
            trail_re = r'(?![a-zA-Z0-9])'

        pattern = r'(?<!::)\b' + simtk_type + trail_re
        empty_text = ''
        for a_line in content.split('\n'):
            original_line = a_line
            if not '#include' in a_line:
                for replacement_meat in re.findall(pattern,a_line):
                    replacement = f'SimTK::{replacement_meat}'
                    a_line = re.sub(pattern, replacement, a_line)
                    if VERBOSE:
                        print(pattern)
                        print(replacement)
            else:
                if simtk_type in a_line:
                    #print(f'\tskipped: {a_line[:10]}')
                    pass ## this seems to be working fine, no need for adding a ton of lines to output
            if VERBOSE and not a_line == original_line:
                print(original_line)
                print(a_line)
            
            empty_text+=a_line + '\n'
        #content = re.sub(pattern, replacement, content)
        content = empty_text
        # Clean up any accidental double prefixes
        content = content.replace(f'SimTK::SimTK::{simtk_type}', f'SimTK::{simtk_type}')
    
    #content = re.sub("(\n\n\n)+",'\n\n',content) ##im adding a bunch of \n for some reason
    while content[-1] == '\n':
        content = content[:-1]
    # Only write if content changed
    if content != original_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True
    return False


def main():
    target_path = Path(TARGET_DIR)
    
    if not target_path.exists():
        print(f"Error: Directory '{TARGET_DIR}' does not exist")
        return
    
    print(f"=== SimTK Namespace Fixer ===")
    print(f"Target directory: {TARGET_DIR}")
    print(f"File extensions: {', '.join(FILE_EXTENSIONS)}")
    print()
    
    # Find all relevant files
    files_to_process = []
    for ext in FILE_EXTENSIONS:
        files_to_process.extend(target_path.rglob(f"*{ext}"))
    
    # Filter to only files with "using namespace SimTK"
    files_with_using = []
    for file_path in files_to_process:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                if 'using namespace SimTK' in f.read():
                    files_with_using.append(file_path)
        except Exception as e:
            print(f"Warning: Could not read {file_path}: {e}")
    
    if not files_with_using:
        print("No files found with 'using namespace SimTK'")
        return
    
    print(f"Found {len(files_with_using)} files with 'using namespace SimTK'")
    print()
    
    # Process each file
    modified_count = 0
    for file_path in files_with_using:
        try:
            if process_file(file_path):
                print(f"✓ {file_path}")
                modified_count += 1
            else:
                print(f"- {file_path} (no changes needed)")
        except Exception as e:
            print(f"✗ {file_path}: {e}")
    
    print()
    print(f"=== Summary ===")
    print(f"Modified {modified_count} files")
    print()
    print("Next steps:")
    print("1. Try to compile")
    print("2. If you get errors about unknown types, add them to SIMTK_TYPES")
    print("3. Run this script again")
    print("4. Repeat until it compiles")
    print()
    print("If it breaks: git restore .")


if __name__ == "__main__":
    main()
