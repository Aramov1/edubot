from pathlib import Path
import sys
_ASSIGNMENT_DIR = Path(__file__).parent.parent.parent
_SCRIPT_DIR     = Path(__file__).parent

sys.path.insert(0, str(_ASSIGNMENT_DIR))    

import sympy as sp
from robot_kinematics import RobotKinematics

def main():
    robot = RobotKinematics()
    jac_matrix = robot._symbolic_jac

    # Print the symbolic Jacobian matrix to the terminal
    sp.pprint(jac_matrix)

    # Save the symbolic Jacobian matrix to a text file in the same directory as this script
    file_path = _SCRIPT_DIR / "jacobian.txt"
    with open(file_path, "w") as f:
        f.write(sp.pretty(jac_matrix, num_columns=10000))
    
    print(f"File saved at: {file_path}")

if __name__ == "__main__":
    main()