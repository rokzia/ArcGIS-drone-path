# segment_probabilities turetu but arejus visu tasku, bet nurodziau tik double kintamuosius nes nemoku isskirti ptikimo tikimybes;
# gauname skirtingus kelius ir juos norime palyginti, galime naudoti sita, jis apskaiciuoja tikimybes kiekviename taske, tdl atsakymas arti vieneto arba arti nulio, o ne objektyvi
def calculate_total_detection_probability_independent(segment_probabilities):
    P_total = 1
    for P_i in segment_probabilities:
        P_total *= (1 - P_i)
    return 1 - P_total
# objektyvi kelio pastebejimo tikimybe, jeigu jau aiskiai pasirinkome kelia
def calculate_total_detection_probability_dependent(segment_probabilities):
    total_probability = 0
    cumulative_no_detection = 1
    for P_i in segment_probabilities:
        total_probability += cumulative_no_detection * P_i
        cumulative_no_detection *= (1 - P_i)
    return total_probability
