import datetime

def week_to_weekly_title(monday: datetime.datetime):
    if monday.weekday() != 0:
        raise RuntimeError("Expected a monday as beginning of week")
    sunday = monday + datetime.timedelta(days=6)
    begin_date_str = monday.strftime('%B ') + str(monday.day)
    end_date_str = str(sunday.day) if monday.month == sunday.month else sunday.strftime('%B ') + str(sunday.day)

    return f"{begin_date_str} - {end_date_str}"
