local b64 = 'ICBnY3M6c2VuZF90ZXh0KDUsICJEUjogcGlsb3QgcmV0b29rIGNvbnRyb2wiKQo='

-- Base64デコード関数
local function base64_decode(data)
    local b = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
    data = string.gsub(data, '[^'..b..'=]', '')
    local pad = #data % 4
    if pad > 0 then
        data = data .. string.rep('=', 4 - pad)
    end
    return (data:gsub('.', function(x)
        if x == '=' then
            return ''
        else
            local r, f = '', (b:find(x) - 1)
            for i = 6, 1, -1 do
                r = r .. (f % 2^i - f % 2^(i-1) > 0 and '1' or '0')
            end
            return r
        end
    end):gsub('%d%d%d?%d?%d?%d?%d?%d?', function(x)
        if #x ~= 8 then
            return ''
        else
            local c = 0
            for i = 1, 8 do
                c = c + (x:sub(i, i) == '1' and 2^(8 - i) or 0)
            end
            return string.char(c)
        end
    end))
end

-- 環境テーブル
local env = {
    print = print, -- 標準ライブラリの関数を追加
}

-- デコードして表示
local decoded_code = base64_decode(b64)
--print("Decoded Code:\n" .. decoded_code)

-- デコードしたコードを実行
local func, err = load(decoded_code, "decoded", "t", env)

function update()

  func()

  return update, 1000
end

return update, 1000

--if func then
--    func()
--else
--    print("Error loading code:", err)
--end
