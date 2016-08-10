-- test IR_TOSTR for char type.
str = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"

for i = 1, 60 do
   x = tostring(string.char(str:byte(i)))
end

y = "X"
assert(x == y, "Got " .. x .. ", expect " .. y)
