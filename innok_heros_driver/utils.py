def check_soc_validity(voltage, soc) -> bool:

    return True

    # FIXME: Disabled for now. This check is too simple and doesn't work for the bms SoC
    
    # upper_limit = 54.0
    # lower_limit = 42.0

    # if voltage > upper_limit:
    #     soc_calculated = 100
    # elif voltage < lower_limit:
    #     soc_calculated = 0
    # else:
    #     soc_calculated = (voltage - lower_limit) / (upper_limit - lower_limit) * 100

    # if abs(soc - soc_calculated) > 10:
    #     return False

    # return True