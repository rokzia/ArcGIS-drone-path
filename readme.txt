Šiame apraše trumpai paaiškinsiu, kaip veikia mūsų automatinis maršrutų radimas.

Kelio paieškai įgyvendinome a star algoritmą. Šis paprastu atveju vertina tašką pagal tai, kiek šis nutolęs nuo starto bei finišo ir pagal galimybę ten eiti.  Kadangi algoritme neatsižvelgiame į koordinatės aukštį, tad mūsų dronas gali skristi bet kur. 
Kiekviena koordinatė turi savo „kainą“. Šią nustatome prie prieš tai minėtų atstumų pridėdami ir pagal radaro stiprį nustatomą dydį. Kadangi nesugebėjome gauti radaro stiprių lygmenų, kuriuos matėme ArcGIS Contents lange, šį dydį skaičiavome formule y = (x+111)/111, nors ir nematėme, jog radaro stipris priartėtų prie 0. Čia x – radaro stipris rastre. 

Susiję metodai kode:
a_star – kelio radimo algoritmas,
update_vertex – koordinačių „kainos“ skaičiavimas

Radę koordinates, kelią taipogi bandome „išlyginti“. Pasinaudodami  Bresenhamo linijos algoritmu (bresenham's line algorithm), tikriname ar koordinatės radaro stipris nėra didesnis nei linijos algoritmo vykdymo metu rastos mažiausios reikšmės. Kol tai nenutinka, šaliname tarpinius taškus ir gauname greitesnį bei įvairių kampų kelią. 

Susiję metodai kode:
line_of_sight - Bresenhamo linijos algoritmas

