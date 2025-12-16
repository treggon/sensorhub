
# Livox Mid-360 Adapter

Use Livox SDK2 (C/C++) to read point clouds and expose to Python.
- SDK: https://github.com/Livox-SDK/Livox-SDK2
- Mid-360 protocol details: https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html

Approaches:
1. Build a small C++ daemon that uses SDK2 to stream binary packets on UDP localhost, then parse in Python and publish samples.
2. Use existing Python projects like OpenPyLivoxv2 if suitable.
