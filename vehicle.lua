g_savedata = g_savedata or {waiting = false, Ynum1 = 0, Ynum2 = 0, Ynum3 = 0}

function onTick()
    -- Set outputs
    output.setNumber(1, g_savedata.Ynum1)
    output.setNumber(2, g_savedata.Ynum2)
    output.setNumber(3, g_savedata.Ynum3)

    -- Check if waiting and no ping

    if g_savedata.waiting then
        return
    end

    -- Send three input numbers
    local Xnum1 = input.getNumber(1)
    local Xnum2 = input.getNumber(2)
    local Xnum3 = input.getNumber(3)
    local url = string.format(
        "http://127.0.0.1:8080/numbers?num1=%d&num2=%d&num3=%d",
        math.floor(Xnum1),
        math.floor(Xnum2),
        math.floor(Xnum3)
	)
    async.httpGet(8080, url)
    g_savedata.waiting = true
end

function httpReply(port, request, response)
    g_savedata.waiting = false
    -- Parse response (e.g., "1,2,3")
    local num1, num2, num3 = response:match("([%-]?%d+),([%-]?%d+),([%-]?%d+)")
    if num1 and num2 and num3 then
        g_savedata.Ynum1 = tonumber(num1)
        g_savedata.Ynum2 = tonumber(num2)
        g_savedata.Ynum3 = tonumber(num3)
    else
        g_savedata.Ynum1 = 0
        g_savedata.Ynum2 = 0
        g_savedata.Ynum3 = 0
    end
end
