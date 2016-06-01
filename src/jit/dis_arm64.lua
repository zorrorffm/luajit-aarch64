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
local bxor = bit.bxor

------------------------------------------------------------------------------
-- Opcode maps
------------------------------------------------------------------------------

local map_adr = { -- PC-rel. addressing
  shift = 31, mask = 1,
  [0] = "adrDBx", "adrpDBx"
}

local map_addsubi = { -- Add/subtract immediate
  shift = 29, mask = 3,
  [0] = "addDNIg", "addsDNIg", "subDNIg", "subsDNIg",
}

local map_logi = { -- Logical immediate
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "andDpNig", "orrDpNig", "eorDpNig", "andsDNig"
    },
    false -- unallocated
  },
  {
    shift = 29, mask = 3,
    [0] = "andDpNig", "orrDpNig", "eorDpNig", "andsDNig"
  }
}

local map_movwi = { -- Move wide immediate
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "movnDWRg", false, "movzDWRg", "movkDWRg"
    }, false -- unallocated
  },
  {
    shift = 29, mask = 3,
    [0] = "movnDWRg", false, "movzDWRg", "movkDWRg"
  },
}

local map_bitf = { -- Bitfield
  shift = 31, mask = 1,
  [0] = {
    shift = 22, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "sbfmDN12w", "bfmDN12w", "ubfmDN12w", false
    }
  },
  {
    shift = 22, mask = 1,
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
        shift = 21, mask = 7,
        [0] = "andDNMSg", "bicDNMSg", "andDNMSg", "bicDNMSg",
        "andDNMSg", "bicDNMSg", "andDNMg", "bicDNMg"
      },
      {
        shift = 21, mask = 7,
        [0] = "orrDNMSg", "ornDNMSg", "orrDNMSg", "ornDNMSg",
        "orrDNMSg", "ornDNMSg", "orrDNMg", "ornDNMg"
      },
      {
        shift = 21, mask = 7,
        [0] = "eorDNMSg", "eonDNMSg", "eorDNMSg", "eonDNMSg",
        "eorDNMSg", "eonDNMSg", "eorDNMg", "eonDNMg"
      },
      {
        shift = 21, mask = 7,
        [0] = "andsDNMSg", "bicsDNMSg", "andsDNMSg", "bicsDNMSg",
        "andsDNMSg", "bicsDNMSg", "andsDNMg", "bicsDNMg"
      }
    },
    false -- unallocated
  },
  {
    shift = 29, mask = 3,
    [0] = {
      shift = 21, mask = 7,
      [0] = "andDNMSg", "bicDNMSg", "andDNMSg", "bicDNMSg",
      "andDNMSg", "bicDNMSg", "andDNMg", "bicDNMg"
    },
    {
      shift = 21, mask = 7,
      [0] = "orrDNMSg", "ornDNMSg", "orrDNMSg", "ornDNMSg",
      "orrDNMSg", "ornDNMSg", "orrDNMg", "ornDNMg"
    },
    {
      shift = 21, mask = 7,
      [0] = "eorDNMSg", "eonDNMSg", "eorDNMSg", "eonDNMSg",
      "eorDNMSg", "eonDNMSg", "eorDNMg", "eonDNMg"
    },
    {
      shift = 21, mask = 7,
      [0] = "andsDNMSg", "bicsDNMSg", "andsDNMSg", "bicsDNMSg",
      "andsDNMSg", "bicsDNMSg", "andsDNMg", "bicsDNMg"
    }
  }
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
  {
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
    }
  }
}

local map_addsubsh = { -- Add/subtract (shifted register)
  shift = 22, mask = 3,
  [0] = map_assh, map_assh, map_assh
}

local map_asex = {
  shift = 22, mask = 3,
  [0] = {
    shift = 29, mask = 3,
    [0] = "addDNMXg", "addsDNMXg", "subDNMXg", "subsDNMXg",
  }
}

local map_addsubex = { -- Add/subtract (extended register)
  shift = 10, mask = 7,
  [0] = map_asex, map_asex, map_asex, map_asex,
  map_asex
}

local map_addsubc = { -- Add/subtract (with carry)
  shift = 10, mask = 63,
  [0] = {
    shift = 29, mask = 3,
    [0] = "adcDNMg", "adcsDNMg", "sbcDNMg", "sbcsDNMg",
  }
}

local map_ccomp = {
  shift = 4, mask = 1,
  [0] = {
    shift = 10, mask = 3,
    [0] = { -- Conditional compare (register)
      shift = 29, mask = 3,
      "ccmnNMVCg", false, "ccmpNMVCg",
    },
    [2] = {  -- Conditional compare (immediate)
      shift = 29, mask = 3,
      "ccmnN5VCg", false, "ccmpN5VCg",
    }
  }
}

local map_csel = { -- Conditional select
  shift = 11, mask = 1,
  [0] = {
    shift = 10, mask = 1,
    [0] = {
      shift = 29, mask = 3,
      [0] = "cselDNMCg", false, "csinvDNMCg", false,
    },
    {
      shift = 29, mask = 3,
      [0] = "csincDNMCg", false, "csnegDNMCg", false,
    }
  }
}

local map_data1s = { -- Data processing (1 source)
  shift = 29, mask = 1,
  [0] = {
    shift = 31, mask = 1,
    [0] = {
      shift = 10, mask = 0x7ff,
      [0] = "rbitDNg", "rev16DNg", "revDNw",
      false, "clzDNg", "clsDNg"
    },
    {
      shift = 10, mask = 0x7ff,
      [0] = "rbitDNg", "rev16DNg", "rev32DNx",
      "revDNx", "clzDNg", "clsDNg"
    }
  }
}

local map_data2s = { -- Data processing (2 source)
  shift = 29, mask = 1,
  [0] = {
    shift = 10, mask = 63,
    false, "udivDNMg", "sdivDNMg", false, false, false, false, "lslvDNMg",
    "lsrvDNMg", "asrvDNMg", "rorvDNMg"  -- TODO: crc32*
  }
}

local map_data3s = { -- Data processing (3 source)
  shift = 29, mask = 7,
  [0] = { -- TODO: Can this be merged with 64-bit?
    shift = 21, mask = 7,
    [0] = {
      shift = 15, mask = 1,
      [0] = "maddDNMAg", "msubDNMAg"
    }
  }, false, false, false,
  {
    shift = 15, mask = 1,
    [0] = {
      shift = 21, mask = 7,
      [0] = "maddDNMAg", "smaddlDxNMwAx", "smulhDNMx", false,
      false, "umaddlDxNMwAx", "umulhDNMx"
    },
    {
      shift = 21, mask = 7,
      [0] = "msubDNMAg", "smsublDxNMwAx", false, false,
      false, "umsublDxNMwAx"
    }
  }
}


local map_datai = { -- Data processing - immediate
  shift = 23, mask = 7,
  [0] = map_adr, map_adr, map_addsubi, false,
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
    shift = 24, mask = 1,
    [0] = map_logsr,
    {
      shift = 21, mask = 1,
      [0] = map_addsubsh, map_addsubex
    }
  },
  {
    shift = 21, mask = 15,
    [0] = map_addsubc, false, map_ccomp, false,
    map_csel, false,
    {
      shift = 30, mask = 1,
      [0] = map_data2s, map_data1s
    },
    false, map_data3s, map_data3s, map_data3s, map_data3s,
    map_data3s, map_data3s, map_data3s, map_data3s
  }
}

local map_lrl = { -- Load register(literal)
  shift = 26, mask = 1,
  [0] = {
    shift = 30, mask = 3,
    [0] = "ldrDwB", "ldrDxB", "ldrswDxB"
  },
  {
    shift = 30, mask = 3,
    [0] = "ldrDsB", "ldrDdB"
  }
}

local map_lsri = {
  shift = 21, mask = 1,
  [0] = {
    shift = 10, mask = 1,
    { -- Load/store register (immediate pre/post-indexed)
      shift = 30, mask = 3,
      [0] = {
        shift = 26, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "strbDwL", "ldrbDwL", "ldrsbDxL", "ldrsbDwL"
        }
      },
      {
        shift = 26, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "strhDwL", "ldrhDwL", "ldrshDxL", "ldrshDwL"
        }
      },
      {
        shift = 26, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "strDwL", "ldrDwL", "ldrswDxL"
        },
        {
          shift = 22, mask = 3,
          [0] = "strDsL", "ldrDsL"
        }
      },
      {
        shift = 26, mask = 1,
        [0] = {
          shift = 22, mask = 3,
          [0] = "strDxL", "ldrDxL"
        },
        {
          shift = 22, mask = 3,
          [0] = "strDdL", "ldrDdL"
        }
      }
    }
  }
}

local map_ls = { -- Loads and stores
  shift = 23, mask = 0x63,
  [0x20] = map_lrl, [0x21] = map_lrl,
  [0x60] = map_lsri, [0x61] = map_lsri

}

local map_datafp = { -- Data processing - SIMD and FP
  shift = 28, mask = 7,
  { -- 001
    shift = 24, mask = 1,
    [0] = {
      shift = 21, mask = 1,
      {
        shift = 10, mask = 3,
        [0] = {
          shift = 12, mask = 1,
          [0] = {
            shift = 13, mask = 1,
            [0] = {
              shift = 14, mask = 1,
              [0] = {
                shift = 15, mask = 1,
                [0] = { -- Conversion between floating-point and integer
                  shift = 31, mask = 1,
                  [0] = {
                    shift = 16, mask = 0xff,
                    [0x20] = "fcvtnsDwNs", [0x21] = "fcvtnuDwNs",
                    [0x22] = "scvtfDsNw", [0x23] = "ucvtfDsNw",
                    [0x24] = "fcvtasDwNs", [0x25] = "fcvtauDwNs",
                    [0x26] = "fmovDwNs", [0x27] = "fmovDsNw",
                    [0x28] = "fcvtpsDwNs", [0x29] = "fcvtpuDwNs",
                    [0x30] = "fcvtmsDwNs", [0x31] = "fcvtmuDwNs",
                    [0x38] = "fcvtzsDwNs", [0x39] = "fcvtzuDwNs",
                    [0x60] = "fcvtnsDwNd", [0x61] = "fcvtnuDwNd",
                    [0x62] = "scvtfDdNw", [0x63] = "ucvtfDdNw",
                    [0x64] = "fcvtasDwNd", [0x65] = "fcvtauDwNd",
                    [0x68] = "fcvtpsDwNd", [0x69] = "fcvtpuDwNd",
                    [0x70] = "fcvtmsDwNd", [0x71] = "fcvtmuDwNd",
                    [0x78] = "fcvtzsDwNd", [0x79] = "fcvtzuDwNd"
                  },
                  {
                    shift = 16, mask = 0xff,
                    [0x20] = "fcvtnsDxNs", [0x21] = "fcvtnuDxNs",
                    [0x22] = "scvtfDsNx", [0x23] = "ucvtfDsNx",
                    [0x24] = "fcvtasDxNs", [0x25] = "fcvtauDxNs",
                    [0x28] = "fcvtpsDxNs", [0x29] = "fcvtpuDxNs",
                    [0x30] = "fcvtmsDxNs", [0x31] = "fcvtmuDxNs",
                    [0x38] = "fcvtzsDxNs", [0x39] = "fcvtzuDxNs",
                    [0x60] = "fcvtnsDxNd", [0x61] = "fcvtnuDxNd",
                    [0x62] = "scvtfDdNx", [0x63] = "ucvtfDdNx",
                    [0x64] = "fcvtasDxNd", [0x65] = "fcvtauDxNd",
                    [0x66] = "fmovDxNd", [0x67] = "fmovDdNx",
                    [0x68] = "fcvtpsDxNd", [0x69] = "fcvtpuDxNd",
                    [0x70] = "fcvtmsDxNd", [0x71] = "fcvtmuDxNd",
                    [0x78] = "fcvtzsDxNd", [0x79] = "fcvtzuDxNd"
                  }
                }
              },
              { -- Floating-point data-processing (1 source)
                shift = 31, mask = 1,
                [0] = {
                  shift = 22, mask = 3,
                  [0] = {
                    shift = 15, mask = 63,
                    [0] = "fmovDNf", "fabsDNf", "fnegDNf",
                    "fsqrtDNf", false, "fcvtDdNs", false, false,
                    "frintnDNf", "frintpDNf", "frintmDNf", "frintzDNf",
                    "frintaDNf", false, "frintxDNf", "frintiDNf",
                  },
                  {
                    shift = 15, mask = 63,
                    [0] = "fmovDNf", "fabsDNf", "fnegDNf",
                    "fsqrtDNf", "fcvtDsNd", false, false, false,
                    "frintnDNf", "frintpDNf", "frintmDNf", "frintzDNf",
                    "frintaDNf", false, "frintxDNf", "frintiDNf",
                  }
                }
              }
            },
            { -- Floating-point compare
              shift = 31, mask = 1,
              [0] = {
                shift = 14, mask = 3,
                [0] = {
                  shift = 23, mask = 1,
                  [0] = {
                    shift = 0, mask = 31,
                    [0] = "fcmpNMf", [8] = "fcmpNZf",
                    [16] = "fcmpeNMf", [24] = "fcmpeNZf",
                  }
                }
              }
            }
          },
          { -- Floating-point immediate
            shift = 31, mask = 1,
            [0] = {
              shift = 5, mask = 31,
              [0] = {
                shift = 23, mask = 1,
                [0] = "fmovDFf"
              }
            }
          }
        },
        { -- Floating-point conditional compare
          shift = 31, mask = 1,
          [0] = {
            shift = 23, mask = 1,
            [0] = {
              shift = 4, mask = 1,
              [0] = "fccmpNMVCf", "fccmpeNMVCf"
            }
          }
        },
        { -- Floating-point data-processing (2 source)
          shift = 31, mask = 1,
          [0] = {
            shift = 23, mask = 1,
            [0] = {
              shift = 12, mask = 15,
              [0] = "fmulDNMf", "fdivDNMf", "faddDNMf", "fsubDNMf",
              "fmaxDNMf", "fminDNMf", "fmaxnmDNMf", "fminnmDNMf",
              "fnmulDNMf"
            }
          }
        },
        { -- Floating-point conditional select
          shift = 31, mask = 1,
          [0] = {
            shift = 23, mask = 1,
            [0] = "fcselDNMCf"
          }
        }
      }
    },
    { -- Floating-point data-processing (3 source)
      shift = 31, mask = 1,
      [0] = {
        shift = 15, mask = 1,
        [0] = {
          shift = 21, mask = 5,
          [0] = "fmaddDNMAf", "fnmaddDNMAf"
        },
        {
          shift = 21, mask = 5,
          [0] = "fmsubDNMAf", "fnmsubDNMAf"
        }
      }
    }
  }
}

local map_br = { -- Branches, exception generating and system instructions
  shift = 29, mask = 7,
  [0] = "bB",
  { -- Compare & branch (immediate)
    shift = 24, mask = 3,
    [0] = "cbzDBg", "cbnzDBg", "tbzDTBw", "tbnzDTBw"
  },
  { -- Conditional branch (immediate)
    shift = 24, mask = 3,
    [0] = {
      shift = 4, mask = 1,
      [0] = {
        shift = 0, mask = 15,
        [0] = "beqB", "bneB", "bhsB", "bloB", "bmiB",
        "bplB", "bvsB", "bvcB", "bhiB", "blsB", "bgeB",
        "bltB", "bgtB", "bleB", "balB"  -- TODO: find better solution ?
      }
    }
  }, false, "blB",
  { -- Compare & branch (immediate)
    shift = 24, mask = 3,
    [0] = "cbzDBg", "cbnzDBg", "tbzDTBx", "tbnzDTBx"
  },
  {
    shift = 24, mask = 3,
    [0] = { -- Exception generation
      shift = 0, mask = 0xe0001f,
      [0x200000] = "brkW"
    },
    { -- System
      shift = 0, mask = 0x3fffff,
      [0x03201f] = "nop"
    },
    { -- Unconditional branch (register)
      shift = 0, mask = 0xfffc1f,
      [0x1f0000] = "brNx", [0x3f0000] = "blrNx",
      [0x5f0000] = "retNx"
    },
  }
}


local map_init = {
  shift = 25, mask = 15,
  [0] = false, false, false, false,
  map_ls, map_datar, map_ls, map_datafp,
  map_datai, map_datai, map_br, map_br,
  map_ls, map_datar, map_ls, map_datafp
}


------------------------------------------------------------------------------

local map_regs = {}

map_regs.x = {[31] = "sp"}
for i=0,30 do map_regs.x[i] = "x"..i; end

map_regs.w = {[31] = "wsp"}
for i=0,30 do map_regs.w[i] = "w"..i; end

map_regs.d = {}
for i=0,31 do map_regs.d[i] = "d"..i; end

map_regs.s = {}
for i=0,31 do map_regs.s[i] = "s"..i; end

local map_cond = {
  [0] = "eq", "ne", "hs", "lo", "mi", "pl", "vs", "vc",
  "hi", "ls", "ge", "lt", "gt", "le", "al",
}

local map_shift = { [0] = "lsl", "lsr", "asr", }

local map_extend = {
  [0] = "uxtb", "uxth", "uxtw", "uxtx",	"sxtb", "sxth",
  "sxtw", "sxtx",
}

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

local function match_reg(p, pat, regnum)
  return map_regs[match(pat, p.."%w-([xwds])")][regnum]
end

local function parse_imm13(op)
  local immN = band(rshift(op, 22), 1)
  local imms = band(rshift(op, 10), 63)
  local immr = band(rshift(op, 16), 63)
  local is64 = band(rshift(op, 31), 1) == 1
  local levels = 0
  local len
  local S, R
  local imm, size = 0, 0

  if immN == 1 then
    len = 6
  else
    local nimms = bit.bnot(imms)
    for i=5,0,-1 do
      if band(rshift(nimms, i), 1) == 1 then len = i; break; end
    end
  end

  for i=1,len do
    levels = lshift(levels, 1) + 1
  end

  S = band(imms, levels)
  R = band(immr, levels)
  size = lshift(1, len)

  for i=1,S+1 do
    imm = lshift(imm, 1) + 1
  end
  if size<64 then
    -- Note: imm is uninterrupted stream of ones, so it's safe.
    imm = bor(rshift(imm, R), lshift(band(imm, lshift(1, R)-1), size-R))
    for i=2,32/size do
      imm = bor(lshift(imm, size), imm)
    end
    -- Note: Bitwise operations have 32-bit output.
    imm = "#0x"..(is64 and tohex(imm):rep(2) or tohex(imm))
  else
    local imm2 = tohex(lshift(band(imm, lshift(1, R)-1), size-32-R))
    imm = "#0x"..imm2..tohex(rshift(imm, R))
  end
  return imm
end

local function parse_immpc(op, name)
  if name == "b" or name == "bl" then
    return arshift(lshift(op, 6), 4)
  elseif name == "adr" or name == "adrp" then
    local immlo = band(rshift(op, 29), 3)
    local immhi = lshift(arshift(lshift(op, 8), 13), 2)
    return bor(immhi, immlo)
  elseif name == "tbz" or name == "tbnz" then
    return lshift(arshift(lshift(op, 13), 18), 2)
  else
    return lshift(arshift(lshift(op, 8), 13), 2)
  end
end

local function parse_fpimm8(op)
  local is64 = band(op, 0x400000) ~= 0
  local imm8 = band(rshift(op, 13), 0xff)
  local E = is64 and 11 or 8
  local norm = is64 and 1023 or 127
  local sign = band(rshift(imm8, 7), 1)
  if sign == 0 then sign = 1 else sign = -1 end
  local b6 = band(rshift(imm8, 6), 1)
  local b54 = band(rshift(imm8, 4), 3)
  local man = band(imm8, 15)
  local exp = bor(b54, lshift(bxor(b6, 1), E-3+2))
  for i=1,E-3 do exp = bor(exp, lshift(b6, i+2)) end
  return sign*bor(man, 16) * math.pow(2, exp-norm-4)
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
  local map_reg
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

  local rt = match(pat, "[gf]")
  if rt then
    if rt == "g" then
      map_reg = band(op, 0x80000000) ~= 0 and map_regs.x or map_regs.w
    else
      map_reg = band(op, 0x400000) ~= 0 and map_regs.d or map_regs.s
    end
  end

  for p in gmatch(pat, ".") do
    local x = nil
    if p == "D" then
      local regnum = band(op, 31)
      x = rt and map_reg[regnum] or match_reg(p, pat, regnum)
    elseif p == "N" then
      local regnum = band(rshift(op, 5), 31)
      x = rt and map_reg[regnum] or match_reg(p, pat, regnum)
    elseif p == "M" then
      local regnum = band(rshift(op, 16), 31)
      x = rt and map_reg[regnum] or match_reg(p, pat, regnum)
    elseif p == "A" then
      local regnum = band(rshift(op, 10), 31)
      x = rt and map_reg[regnum] or match_reg(p, pat, regnum)
    elseif p == "B" then
      local addr = ctx.addr + pos + parse_immpc(op, name)
      ctx.rel = addr
      x = "0x"..tohex(addr)
    elseif p == "T" then
      x = bor(band(rshift(op, 26), 32), band(rshift(op, 19), 31))
    elseif p == "V" then
      x = band(op, 15)
    elseif p == "C" then
      x = map_cond[band(rshift(op, 12), 15)]
    elseif p == "W" then
      x = band(rshift(op, 5), 0xffff) -- immediate 16
    elseif p == "L" then
      local rn = map_regs.x[band(rshift(op, 5), 31)]
      local imm9 = arshift(lshift(op, 11), 23)
      if band(op, 0x800) ~= 0 then
	x = "["..rn..", #"..imm9.."]!"
      else
	x = "["..rn.."], #"..imm9
      end
    elseif p == "I" then
      x = band(rshift(op, 22), 3)
      if x == 1 then
        x = band(rshift(op, 10), 0x0fff)..", lsl #12"
      else
        x = band(rshift(op, 10), 0x0fff)
      end
    elseif p == "i" then
      x = parse_imm13(op)
    elseif p == "1" then
      x = band(rshift(op, 16), 63)
    elseif p == "2" then
      x = band(rshift(op, 10), 63)
    elseif p == "5" then
      x = band(rshift(op, 16), 31)
    elseif p == "S" then
      x = map_shift[band(rshift(op, 22), 3)].." #"..band(rshift(op, 10), 63)
    elseif p == "X" then
      x = map_extend[band(rshift(op, 13), 7)].." #"..band(rshift(op, 10), 7)
    elseif p == "R" then
      x = band(rshift(op,21), 3)
      if x == 0 then x = nil
      else x = "lsl #"..x*16 end
    elseif p == "g" or p == "f" or p == "x" or p == "w" or
           p == "p" or p == "d" or p == "s" then
      -- These are handled in D/N/M/A.
    elseif p == "Z" then
      x = 0
    elseif p == "F" then
      x = parse_fpimm8(op)
    elseif p == "g" or p == "f" or p == "x" or p == "w" or
           p == "p" or p == "d" or p == "s" then
      -- These are handled in D/N/M/A.
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
  if r < 32 then return map_reg.x[r] end
  return map_reg.d[r-32]
end

-- Public module functions.
return {
  create = create,
  disass = disass,
  regname = regname
}
