-- test IR_XLOAD for int type.
ffi = require("ffi")

ctype1 = ffi.typeof("int[10]")
ctype2 = ffi.typeof("struct { int arr[10]; }")
z = 12

-- initialize all x's element with z
x = ctype1(z)
y = ctype2()

for i = 1, 100 do
  y.arr = x
end

assert(y.arr[0] == z, "Got " .. y.arr[0] .. ", expect " .. z)
assert(y.arr[9] == z, "Got " .. y.arr[9] .. ", expect " .. z)