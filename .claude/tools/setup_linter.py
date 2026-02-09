#!/usr/bin/env python3
import os

# Cấu hình Flake8 chuẩn cho ROS 2
FLAKE8_CONFIG = """[flake8]
extend-ignore = B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202,CNL100,E203,E501,Q000
import-order-style = pep8
max-line-length = 100
statistics = True
count = True
show-source = True
"""

# Cấu hình Clang-Format chuẩn 
CLANG_FORMAT_CONFIG = """---
Language: Cpp
BasedOnStyle: Google
AccessModifierOffset: -2
AlignAfterOpenBracket: AlwaysBreak
ColumnLimit: 100
IndentWidth: 2
...
"""

def main():
    root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # Ghi setup.cfg
    with open(os.path.join(root_dir, 'setup.cfg'), 'w') as f:
        f.write(FLAKE8_CONFIG)
    print("Đã tạo setup.cfg (Flake8 Standard)")
    
    # Ghi .clang-format
    with open(os.path.join(root_dir, '.clang-format'), 'w') as f:
        f.write(CLANG_FORMAT_CONFIG)
    print("Đã tạo .clang-format (Google/ROS C++ Style)")

if __name__ == '__main__':
    main()
