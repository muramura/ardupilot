-- create and initialise parameters
local PARAM_TABLE_KEY = 99          -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, 'TONEALARM_', 1), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'START', 0), 'could not add TONEALARM_START param')     -- TONEALARM_START 0:stop, 1:start


-- called at 10hz
function update()
  local TONEALARM_START = Parameter("TONEALARM_START")
  local tonealarm_start = TONEALARM_START:get()
  if tonealarm_start > 0 then
    gcs:send_text(0, "### TONEALARM_START 2")
    TONEALARM_START:set(0)
    notify:play_tune("MFT240L8O4aO5dcO4aO5dcO4aO5dcL16dcdcdcdc")
--  else
--    gcs:send_text(0, "### TONEALARM_START 3")
  end

  return update, 3000 -- reschedules the loop
end


gcs:send_text(0, "### TONEALARM_START")
return update(), 100

