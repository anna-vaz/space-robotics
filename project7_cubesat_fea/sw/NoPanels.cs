// Makes CubeSat_3U_v3_nopanels.SLDASM: a copy of the v3 assembly with the 4 solar panels suppressed,
// for the buckling sensitivity case "braces without help from the bonded panels". Read-only on v3.
using System;
using System.Linq;
using SolidWorks.Interop.sldworks;

public static class NoPanels
{
    const string Dir = @"D:\cad free projects\CubeSat 3U\";
    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        string src = Dir + "CubeSat_3U_v3.SLDASM", dst = Dir + "CubeSat_3U_v3_nopanels.SLDASM";
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(src) ?? (ModelDoc2)sw.OpenDoc6(src, 2, 0, "", ref e, ref w);
        bool ok = doc.Extension.SaveAs(dst, 0, 2 /* copy */, null, ref e, ref w);   // v3 stays the open document
        Console.WriteLine("copy saved: " + ok + " (errors " + e + ")");
        var cp = (ModelDoc2)sw.OpenDoc6(dst, 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(cp.GetTitle(), false, 0, ref e);
        double m0 = ((MassProperty)cp.Extension.CreateMassProperty()).Mass;
        int n = 0;
        foreach (Component2 c in (object[])((AssemblyDoc)cp).GetComponents(true))
            if (c.Name2.StartsWith("solar_panel")) { c.SetSuppression2(0); n++; }
        cp.EditRebuild3();
        double m1 = ((MassProperty)cp.Extension.CreateMassProperty()).Mass;
        cp.Save3(1, ref e, ref w);
        Console.WriteLine(string.Format("suppressed {0} panels: mass {1:F3} -> {2:F3} kg; gravity scale for equal load = {3:F4}",
            n, m0, m1, m0 / m1));
    }
}
