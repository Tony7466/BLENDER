from coverage_report import parse, report_as_html

parse(".", "./analysis")
report_as_html("./analysis", "./report")
