-- test upvalue store
-- TODO add more cases on different type: number, primitive, non-primitive
function setgen ()
  local x = 0.3
  return function (b)
           for i = 1, 100 do
             x = b
           end
         return x
         end
end

set = setgen()
x = set(2)
y = 2

assert(x == y, "Got " .. x .. ", expect " .. y)
