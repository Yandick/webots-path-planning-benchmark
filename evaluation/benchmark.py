"""Run the revised 2D benchmark. Physical evaluation: webots_benchmark.py."""
import sys
from revised_benchmark import main
if __name__ == "__main__":
    # Keep the historical entry point, while making the backend explicit.
    if "--run-all" in sys.argv:
        sys.argv.remove("--run-all")
    print("Backend: kinematic_2d. Webots physics uses evaluation/webots_benchmark.py.")
    main()
