// Registers the project's material library folder with SOLIDWORKS and assigns the custom
// smeared / dummy materials (library "CubeSat3U") to the generated interior parts.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class MatFix
{
    const string Dir = @"D:\cad free projects\CubeSat 3U";

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int key = (int)swUserPreferenceStringValue_e.swFileLocationsMaterialDatabases;
        string cur = sw.GetUserPreferenceStringValue(key);
        Console.WriteLine("material database folders: " + cur);
        if (!cur.Split(';').Any(s => s.TrimEnd('\\').Equals(Dir, StringComparison.OrdinalIgnoreCase)))
        {
            sw.SetUserPreferenceStringValue(key, cur.TrimEnd(';') + ";" + Dir);
            Console.WriteLine("added " + Dir);
        }
        var ci = CultureInfo.InvariantCulture;
        foreach (var line in File.ReadAllLines(Path.Combine(outDir, "..", "cad", "interior_spec.txt")).Where(l => l.StartsWith("PART|")))
        {
            var f = line.Split('|');
            int n = f.Length;
            string mat = f[n - 3]; double target = double.Parse(f[n - 1], ci);
            if (mat == "6061-T6 (SS)") continue;
            string path = Path.Combine(Dir, f[1]);
            int e = 0, w = 0;
            var doc = (ModelDoc2)sw.OpenDoc6(path, 1, 0, "", ref e, ref w);
            ((PartDoc)doc).SetMaterialPropertyName2("", "CubeSat3U", mat);
            string db; string got = ((PartDoc)doc).GetMaterialPropertyName2("", out db);
            doc.EditRebuild3();
            double m = ((MassProperty)doc.Extension.CreateMassProperty()).Mass * 1e3;
            bool ok = Math.Abs(m - target) < 0.01;
            if (ok) doc.Save3(1, ref e, ref w);
            Console.WriteLine(string.Format("{0,-22} {1,9:F3} g (target {2,9:F3}) '{3}' [{4}] {5}", f[1], m, target, got, db, ok ? "OK, saved" : "MISMATCH"));
            sw.CloseDoc(doc.GetTitle());
        }
    }
}
