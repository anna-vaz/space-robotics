// Renders the full CubeSat assembly: closed view and an "inside" view with the solar panels,
// shear panels and one side frame hidden. Visibility is restored afterwards; nothing is saved.
using System;
using System.IO;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;

public static class ShotInside
{
    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.OpenDoc6(@"D:\cad free projects\CubeSat 3U\CubeSat_3U.SLDASM", 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        string dir = Path.Combine(outDir, "shots");
        Directory.CreateDirectory(dir);
        Snap(doc, "*Trimetric", Path.Combine(dir, "cubesat_closed.bmp"));
        var hidden = new List<Component2>();
        foreach (Component2 c in (object[])((AssemblyDoc)doc).GetComponents(true))
        {
            string n = c.Name2;
            if (n.StartsWith("solar_panel") || n.StartsWith("shear_panel") || n == "side_frame-2")
            { c.Visible = 0; hidden.Add(c); }
        }
        Snap(doc, "*Trimetric", Path.Combine(dir, "cubesat_inside.bmp"));
        Snap(doc, "*Front", Path.Combine(dir, "cubesat_inside_side.bmp"));
        foreach (var c in hidden) c.Visible = 1;
        doc.ShowNamedView2("*Trimetric", -1); doc.ViewZoomtofit2();
    }

    static void Snap(ModelDoc2 doc, string view, string f)
    {
        doc.ShowNamedView2(view, -1);
        doc.ViewZoomtofit2();
        doc.GraphicsRedraw2();
        doc.SaveBMP(f, 1600, 1200);
        Console.WriteLine(f);
    }
}
