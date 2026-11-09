// Renders the side frame to BMP images (read-only, the part is not modified).
using System;
using System.IO;
using SolidWorks.Interop.sldworks;

public static class Shot
{
    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.OpenDoc6(@"D:\cad free projects\CubeSat 3U\side_frame.SLDPRT", 1, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        doc.SetUserPreferenceToggle(77, false);   // swDisplaySketches off (doc-level display)
        doc.ClearSelection2(true);
        string dir = Path.Combine(outDir, "shots");
        Directory.CreateDirectory(dir);
        string[] views = { "*Isometric", "*Trimetric" };
        foreach (string v in views)
        {
            doc.ShowNamedView2(v, -1);
            doc.ViewZoomtofit2();
            doc.GraphicsRedraw2();
            string f = Path.Combine(dir, "side_frame_" + v.Trim('*').ToLower() + ".bmp");
            doc.SaveBMP(f, 2000, 1400);
            Console.WriteLine(f);
        }
        // close-up of one end: holes in both flanges, foot, end band
        doc.ShowNamedView2("*Isometric", -1);
        doc.ViewZoomTo2(-0.005, -0.005, -0.005, 0.015, 0.105, 0.045);
        doc.GraphicsRedraw2();
        string fc = Path.Combine(dir, "side_frame_end_closeup.bmp");
        doc.SaveBMP(fc, 2000, 1400);
        Console.WriteLine(fc);
        doc.ShowNamedView2("*Isometric", -1);
        doc.ViewZoomtofit2();
    }
}
