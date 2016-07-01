x = 0
num = "10.0"

for i = 1, 100 do
  x = num + 1
end

y = 11

assert(x == y, "Got " .. x .. ", expect " .. y)

-- TODO add more test cases to cover more branch in STRTO ir implementation
