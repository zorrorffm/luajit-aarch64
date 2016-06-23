x = 0

for i = -50, 51 do
  x = x + math.sinh(i)
end

y = math.sinh(51)

assert(x == y, "Got " .. x .. ", expect " .. y)
