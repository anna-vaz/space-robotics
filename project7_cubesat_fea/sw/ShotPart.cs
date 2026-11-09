// Renders one part (path in env SHOT_PART) to results/shots/<name>_isometric.png-ready BMP. Read-only.
using System;
using System.IO;
using SolidWorks.Interop.sldworks;

public static class ShotPart
{
    public static void Run(string outDir)
    {
        string path = System.Environment.GetEnvironmentVariable("SHOT_PART");
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        int type = path.ToUpper().EndsWith(".SLDASM") ? 2 : 1;
        var doc = (ModelDoc2)sw.OpenDoc6(path, type, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        string dir = Path.Combine(outDir, "shots");
        Directory.CreateDirectory(dir);
        foreach (string v in new[] { "*Isometric", "*Trimetric" })
        {
            doc.ShowNamedView2(v, -1);
            doc.ViewZoomtofit2();
            doc.GraphicsRedraw2();
            string f = Path.Combine(dir, Path.GetFileNameWithoutExtension(path) + "_" + v.Trim('*').ToLower() + ".bmp");
            doc.SaveBMP(f, 2000, 1400);
            Console.WriteLine(f);
        }
        string zoom = System.Environment.GetEnvironmentVariable("SHOT_ZOOM");   // x1,y1,z1,x2,y2,z2 mm
        if (!string.IsNullOrEmpty(zoom))
        {
            var z = Array.ConvertAll(zoom.Split(','), v => double.Parse(v, System.Globalization.CultureInfo.InvariantCulture) / 1000);
            doc.ShowNamedView2("*Isometric", -1);
            doc.ViewZoomTo2(z[0], z[1], z[2], z[3], z[4], z[5]);
            doc.GraphicsRedraw2();
            string f = Path.Combine(dir, Path.GetFileNameWithoutExtension(path) + "_closeup.bmp");
            doc.SaveBMP(f, 2000, 1400);
            Console.WriteLine(f);
            doc.ShowNamedView2("*Trimetric", -1); doc.ViewZoomtofit2();
        }
    }
}
