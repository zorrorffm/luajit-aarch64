----------------------------------------------------------------------------
-- LuaJIT ARM64 disassembler module.
--
-- Copyright (C) 2005-2016 Mike Pall. All rights reserved.
-- Released under the MIT license. See Copyright Notice in luajit.h
----------------------------------------------------------------------------
-- This is a helper module used by the LuaJIT machine code dumper module.
--
-- It disassembles most user-mode AArch64 instructions
-- NYI: Advanced SIMD and VFP instructions.
------------------------------------------------------------------------------

local type = type
local sub, byte, format = string.sub, string.byte, string.format
local match, gmatch, gsub = string.match, string.gmatch, string.gsub
local concat = table.concat
local bit = require("bit")
local band, bor, ror, tohex = bit.band, bit.bor, bit.ror, bit.tohex
local lshift, rshift, arshift = bit.lshift, bit.rshift, bit.arshift

------------------------------------------------------------------------------
-- Opcode maps
------------------------------------------------------------------------------

local map_adr = { -- PC-rel. addressing
  shift = 31, mask = 1,
  [0] = "adrDBx", "adrpDBx"
}

local map_addsubi = { -- Add/subtract immediate
  shift = 29, mask = 7,
  [0] = "addDpNIw", "addsDpNIw", "subDpNIw", "subsDpNIw",
  "addDpNIx", "addsDpNIx", "subDpNIx", "subsDpNIx",
}

local map_logi = { -- Logical immediate
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "andDpNiw", "orrDpNiw", "eorDpNiw", "andsDNiw"
    },
    false -- unallocated
  },
  {
    shift = 29, mask = 3,
    [0] = "andDpNix", "orrDpNix", "eorDpNix", "andsDNix"
  }
}

local map_movwi = { -- Move wide immediate
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "movnDWw", false, "movzDWw", "movkDWw"  -- TODO: check DWg or DWRg
    }, false -- unallocated
  },
  {
    shift = 29, mask = 3,
    [0] = "movnDWx", false, "movzDWx", "movkDWx"
  },
}

local map_bitf = { -- Bitfield
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "sbfmDN12w", "bfmDN12w", "ubfmDN12w", false
    },
    false
  },
  {
    shift = 22, mask = 1,
    [0] = false,
    {
      shift = 29, mask = 3,
      [0] = "sbfmDN12x", "bfmDN12x", "ubfmDN12x", false
    }
  }
}

local map_logsr = { -- Logical (shifted register)
  shift = 31, mask = 1,
  [0] = {
    shift = 15, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = {
        shift = 21, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "andDNMSg", "andDNMSg", "andDNMSg", "andDNMg"
        },
        {
          shift = 22, mask = 3,
          [0] = "bicDNMSg", "bicDNMSg", "bicDNMSg", "bicDNMg"
        }
      },
      {
        shift = 21, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "orrDNMSg", "orrDNMSg", "orrDNMSg", "orrDNMg"
        },
        {
          shift = 22, mask = 3,
          [0] = "ornDNMSg", "ornDNMSg", "ornDNMSg", "ornDNMg"
        }
      },
      {
        shift = 21, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "eorDNMSg", "eorDNMSg", "eorDNMSg", "eorDNMg"
        },
        {
          shift = 22, mask = 3,
          [0] = "eonDNMSg", "eonDNMSg", "eonDNMSg", "eonDNMg"
        }
      },
      {
        shift = 21, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "andsDNMSg", "andsDNMSg", "andsDNMSg", "andsDNMg"
        },
        {
          shift = 22, mask = 3,
          [0] = "bicsDNMSg", "bicsDNMSg", "bicsDNMSg", "bicsDNMg"
        }
      }
    },
    false -- unallocated
  },
  false -- 64 bit operations not implemented
}

local map_assh = {
  shift = 31, mask = 1,
  [0] = {
    shift = 15, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = {
        shift = 22, mask = 3,
        [0] = "addDNMSg", "addDNMSg", "addDNMSg", "addDNMg"
      },
      {
        shift = 22, mask = 3,
        [0] = "addsDNMSg", "addsDNMSg", "addsDNMSg", "addsDNMg"
      },
      {
        shift = 22, mask = 3,
        [0] = "subDNMSg", "subDNMSg", "subDNMSg", "subDNMg"
      },
      {
        shift = 22, mask = 3,
        [0] = "subsDNMSg", "subsDNMSg", "subsDNMSg", "subsDNMg"
      },
    },
    false -- unallocated
  },
  false -- 64 bit operations not implemented
}

local map_addsubsh = { -- Add/subtract (shifted register)
  shift = 22, mask = 3,
  [0] = map_assh, map_assh, map_assh, false
}

local map_asex = {
  shift = 22, mask = 3,
  [0] = {
    shift = 31, mask = 1,  -- is this check needed?
    [0] = {
      shift = 29, mask = 3,
      [0] = "addDNMXg", "addsDNMXg", "subDNMXg", "subsDNMXg"
    }, 
    false
  }, 
  false, false, false
}


local map_addsubex = { -- Add/subtract (extended register)
  shift = 10, mask = 7,
  [0] = map_asex, map_asex, map_asex, map_asex,
  map_asex, false, false, false
}

local map_addsubc = { -- Add/subtract (with carry)
  shift = 10, mask = 63,
  [0] = {
    shift = 29, mask = 7,
    [0] = "adcDNMw", "adcsDNMw", "sbcDNMw", "sbcsDNMw",
    "adcDNMx", "adcsDNMx", "sbcDNMx", "sbcsDNMx",
  }        
}

local map_ccompr = { -- Conditional compare (register)

}

local map_ccompi = { -- Conditional compare (immediate)

}

local map_csel = { -- Conditional select

}

local map_data1s = { -- Data processing (1 source)

}

local map_data2s = { -- Data processing (2 source)

}

local map_data3s = { -- Data processing (3 source)

}


local map_datai = { -- Data processing - immediate
  shift = 23, mask = 7,
  [0] = map_adr, map_adr, map_addsubi, map_addsubi,
  map_logi, map_movwi, map_bitf,
  {
    shift = 15, mask = 0x1c0c1,
    [0] = "extrDNM2w", [0x10080] = "extrDNM2x",
    [0x10081] = "extrDNM2x"
  }
}

local map_datar = { -- Data processing - register
  shift = 28, mask = 1,
  [0] = {
    shift = 21, mask = 15,
    [0] = map_logsr, map_logsr, map_logsr, map_logsr,
    map_logsr, map_logsr, map_logsr, map_logsr,
    map_addsubsh, map_addsubex, map_addsubsh, map_addsubex,
    map_addsubsh, map_addsubex, map_addsubsh, map_addsubex
  },
  {
    shift = 21, mask = 15,
    [0] = map_addsubc, false, 
    {
      shift = 11, mask = 1,
      [0] = map_ccompr, map_ccompi
    },
    false, map_csel, false,
    {
      shift = 30, mask = 1,
      [0] = map_data2s, map_data1s
    },
    false, map_data3s, map_data3s, map_data3s, map_data3s,
    map_data3s, map_data3s, map_data3s, map_data3s
  }
}

local map_ls = { -- Loads and stores

}

local map_datafp = { --Data processing - SIMD and FP

}

local map_br = { -- Branches, exception generating and system instructions

}


local map_init = {
  shift = 25, mask = 15,
  [0] = false, false, false, false,
  map_ls, map_datar, map_ls, map_datafp,
  map_datai, map_datai, map_br, map_br,
  map_ls, map_datar, map_ls, map_datafp
}


------------------------------------------------------------------------------

local map_gpr = {
  [0] = "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
  "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
}

local map_cond = {
  [0] = "eq", "ne", "hs", "lo", "mi", "pl", "vs", "vc",
  "hi", "ls", "ge", "lt", "gt", "le", "al",
}

local map_shift = { [0] = "lsl", "lsr", "asr", }

------------------------------------------------------------------------------


-- Output a nicely formatted line with an opcode and operands.
local function putop(ctx, text, operands)
  local pos = ctx.pos
  local extra = ""
  if ctx.rel then
    local sym = ctx.symtab[ctx.rel]
    if sym then
      extra = "\t->"..sym
    elseif band(ctx.op, 0x0e000000) ~= 0x0a000000 then  --TODO: check
      extra = "\t; 0x"..tohex(ctx.rel)
    end
  end
  if ctx.hexdump > 0 then
    ctx.out(format("%08x  %s  %-5s %s%s\n",
      ctx.addr+pos, tohex(ctx.op), text, concat(operands, ", "), extra))
  else
    ctx.out(format("%08x  %-5s %s%s\n",
      ctx.addr+pos, text, concat(operands, ", "), extra))
  end
  ctx.pos = pos + 4
end

-- Fallback for unknown opcodes.
local function unknown(ctx)
  return putop(ctx, ".long", { "0x"..tohex(ctx.op) })
end


-- Disassemble a single instruction.
local function disass_ins(ctx)
  local pos = ctx.pos
  local b0, b1, b2, b3 = byte(ctx.code, pos+1, pos+4)
  local op = bor(lshift(b3, 24), lshift(b2, 16), lshift(b1, 8), b0) -- order depends on endian
  local operands = {}
  local suffix = ""
  local last, name, pat
  local vr
  ctx.op = op
  ctx.rel = nil

  local opat
  opat = map_init[band(rshift(op, 25), 15)]
  while type(opat) ~= "string" do
    if not opat then return unknown(ctx) end

    opat = opat[band(rshift(op, opat.shift), opat.mask)] or opat._ -- calculating index of map
  end
  name, pat = match(opat, "^([a-z0-9]*)(.*)")
  if sub(pat, 1, 1) == "." then
    local s2, p2 = match(pat, "^([a-z0-9.]*)(.*)")
    suffix = suffix..s2
    pat = p2
  end

  for p in gmatch(pat, ".") do
    local x = nil
    if p == "D" then
      x = map_gpr[band(op, 31)]
    elseif p == "N" then
      x = map_gpr[band(rshift(op, 5), 31)]
    elseif p == "M" then
      x = map_gpr[band(rshift(op, 16), 31)]
    elseif p == "B" then

    elseif p == "W" then
      x = band(rshift(op, 5), 0xffff) -- immediate 16
    elseif p == "x" then
    --  x = band(rshift(op, 16), 31) + 1
    elseif p == "p" then

    elseif p == "I" then
      x = band(rshift(op, 10), 0x0fff) -- immediate 12
    elseif p == "i" then
    -- Decode bitmask
    elseif p == "g" then

    elseif p == "1" then
      x = band(rshift(op, 16), 63)
    elseif p == "2" then
      x = band(rshift(op, 10), 63)
    elseif p == "w" then

    elseif p == "S" then
    -- shift register
    elseif p == "X" then
    -- extended reg ?
    else
      assert(false)
    end
    if x then
      --last = x
      if type(x) == "number" then x = "#"..x end
      operands[#operands+1] = x
    end
  end

  return putop(ctx, name..suffix, operands)
end


------------------------------------------------------------------------------

-- Disassemble a block of code.
local function disass_block(ctx, ofs, len)
  if not ofs then ofs = 0 end
  local stop = len and ofs+len or #ctx.code
  ctx.pos = ofs
  ctx.rel = nil
  while ctx.pos < stop do disass_ins(ctx) end
end

-- Extended API: create a disassembler context. Then call ctx:disass(ofs, len).
local function create(code, addr, out)
  local ctx = {}
  ctx.code = code
  ctx.addr = addr or 0
  ctx.out = out or io.write
  ctx.symtab = {}
  ctx.disass = disass_block
  ctx.hexdump = 8
  return ctx
end

-- Simple API: disassemble code (a string) at address and output via out.
local function disass(code, addr, out)
  create(code, addr, out):disass()
end

-- Return register name for RID.
local function regname(r)
  if r < 16 then return map_gpr[r] end
  return "d"..(r-16)
end

-- Public module functions.
return {
  create = create,
  disass = disass,
  regname = regname
}
