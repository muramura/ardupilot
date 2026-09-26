-- hello_anne.lua
-- Periodically notify "HELLO ANNE" to GCS every 3 seconds via ROMFS
local SCRIPT_NAME = "HELLO ANNE"
local INTERVAL_MS = 3000 -- 3000ms = 3 seconds

local count = 0

function update()
    count = count + 1
    gcs:send_text(6, string.format("HELLO ANNE (%d)", count))
    return update, INTERVAL_MS
end

gcs:send_text(6, "ROMFS: hello_anne.lua loaded successfully!")
return update, INTERVAL_MS
