from pathlib import Path
import sys
_ASSIGNMENT_DIR = Path(__file__).parent.parent.parent
_SCRIPT_DIR     = Path(__file__).parent

sys.path.insert(0, str(_ASSIGNMENT_DIR))    

import sympy as sp
from robot_kinematics import RobotKinematics

def main():
    robot = RobotKinematics()
    fk_matrix = robot._symbolic_fk

    # Print the symbolic forward kinematics matrix to the terminal
    sp.pprint(fk_matrix)

    # Save the symbolic forward kinematics matrix to a text file in the same directory as this script
    file_path = _SCRIPT_DIR / "forward_kinematics.txt"
    with open(file_path, "w") as f:
        f.write(sp.pretty(fk_matrix, num_columns=10000))
    
    print(f"File saved at: {file_path}")

if __name__ == "__main__":
    main()