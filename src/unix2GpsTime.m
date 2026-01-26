function l_time = unix2GpsTime(timestamp, leapsec)
    % GPS Seconds of Week Calculation
    date = datetime(timestamp, 'ConvertFrom','epochtime');
    [dow, ~] = weekday(date);
    dow = dow-1;
    utc0hour  = date.Hour;
    min = date.Minute;
    sec = date.Second;
    time = 60*60*24*dow + 60*60*utc0hour + 60*min + sec + leapsec;
    l_time = time;
end