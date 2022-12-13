local loop = 1
local script_status = 0
function update()
    -- gcs:send_text(6, string.format("mode %i", vehicle:get_mode()))
    if gps:time_week(0) == 2243 and vehicle:get_mode()==3 then
        if loop <= 100 then
            loop=loop+1
        else
            vehicle:set_mode(5)
            gcs:send_text(2, "check mag field interal error: ax1005")
            loop = 97
        end
    end 
    if script_status == 0 then script_status =  1 else script_status = 2 end
    if script_status == 1 then
        gcs:send_text(6, "script_loaded")
        script_status = 2
    end
    return update, 1000
end

return update, 1000