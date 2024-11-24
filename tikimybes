import arcpy
import numpy as np

#rcs-reflektavimo savybė

def skaiciuoti_snr(galia, stiprinimas, bangu_ilgis, rcs, atstumas):
    return (galia * stiprinimas**2 * bangu_ilgis**2 * rcs) / ((4 * np.pi)**3 * atstumas**4)

#slenksti - snr, aka signal-to-noise ratio
def aptikimo_tikimybe(snr, slenkstis=10):
    return 1 - np.exp(-snr / slenkstis)

radaro_sluoksnis = "Radarai"
uav_sluoksnis = "UAVMaršrutas"

radaru_duomenys = []
with arcpy.da.SearchCursor(radaro_sluoksnis, ["SHAPE@XY", "Galia", "Stiprinimas", "Bangos_Ilgis"]) as kursorius:
    for eilute in kursorius:
        radaru_duomenys.append({
            "vieta": eilute[0],
            "galia": eilute[1],
            "stiprinimas": eilute[2],
            "bangu_ilgis": eilute[3]
        })

uav_tikimybes = []
with arcpy.da.SearchCursor(uav_sluoksnis, ["SHAPE@XY", "RCS", "Aukštis"]) as kursorius:
    for uav_eilute in kursorius:
        uav_vieta = uav_eilute[0]
        rcs = uav_eilute[1]
        aukstis = uav_eilute[2]
        bendra_tikimybe = 0

        for radaras in radaru_duomenys:
            radaro_vieta = radaras["vieta"]
            atstumas = np.sqrt((uav_vieta[0] - radaro_vieta[0])**2 +
                               (uav_vieta[1] - radaro_vieta[1])**2 + 
                               aukstis**2)
            snr = skaiciuoti_snr(radaras["galia"], radaras["stiprinimas"], radaras["bangu_ilgis"], rcs, atstumas)
            tikimybe = aptikimo_tikimybe(snr)
            bendra_tikimybe += tikimybe
        
        uav_tikimybes.append(bendra_tikimybe)

with arcpy.da.UpdateCursor(uav_sluoksnis, ["Aptikimo_Tikimybe"]) as kursorius:
    for i, eilute in enumerate(kursorius):
        eilute[0] = uav_tikimybes[i]
        kursorius.updateRow(eilute)

# Simbolikos nustatymas
lygis = arcpy.mp.ArcGISProject("CURRENT").activeMap.listLayers("UAVMaršrutas")[0]
sym = arcpy.management.ApplySymbologyFromLayer(lygis, "Graduated Colors")
arcpy.ApplySymbologyFromLayer_management(
    lyr=sym,
    symbologyFields="Aptikimo_Tikimybe"
)
print("Tikimybės apskaičiuotos, sluoksnis atnaujintas ir vizualizuotas.")
