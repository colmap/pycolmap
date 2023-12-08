git ls-tree --full-tree -r --name-only HEAD . | grep ".*\(\.cc\|\.h\|\.hpp\|\.cpp\|\.cu\)$" | xargs clang-format -i
