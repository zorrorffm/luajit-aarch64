x = 1

for i = 1, 100 do
  x = x * i
end

y = 0

assert(x == y, "Got " .. x .. ", expect " .. y)
