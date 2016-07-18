-- test upvalue load
-- TODO add more cases on different type: number, primitive, non-primitive
function setgen ()
  local x = {1}
  return function (b)
           for i = 1, 100 do
             b = x[1]
           end
         return b
         end
end

set = setgen(0)
x = set(1)
y = 1

assert(x == y, "Got " .. x .. ", expect " .. y)
