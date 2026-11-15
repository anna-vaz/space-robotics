// Final CAD deliverables for the repo (read-only on the source CAD):
//   1. Pack and Go of CubeSat_3U_v5.SLDASM + every referenced part -> <repo>/project7_cubesat_fea/cad/solidworks_v5/
//   2. neutral STEP AP214 of the assembly -> cad/CubeSat_3U_v5.step
//   3. renders: closed, inside (solar panels, shear panels, side_frame-2 hidden) -> results/shots/v5_*.bmp
using System;
using System.IO;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;

public static class FinalCad
{
    public static void Run(string outDir)
    {
        string src = @"D:\cad free projects\CubeSat 3U\CubeSat_3U_v5.SLDASM";
        string cad = Path.GetFullPath(Path.Combine(outDir, "..", "cad"));
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.OpenDoc6(src, 2, 0, "", ref e, ref w);
        Console.WriteLine("open err " + e + ", warn " + w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);

        // renders
        string dir = Path.Combine(outDir, "shots");
        Directory.CreateDirectory(dir);
        Snap(doc, "*Trimetric", Path.Combine(dir, "v5_closed.bmp"));
        var hidden = new List<Component2>();
        foreach (Component2 c in (object[])((AssemblyDoc)doc).GetComponents(true))
        {
            string n = c.Name2;
            if (n.StartsWith("solar_panel") || n.StartsWith("shear_panel") || n == "side_frame-2")
            { c.Visible = 0; hidden.Add(c); }
        }
        Snap(doc, "*Trimetric", Path.Combine(dir, "v5_inside.bmp"));
        Snap(doc, "*Front", Path.Combine(dir, "v5_inside_front.bmp"));
        foreach (var c in hidden) c.Visible = 1;
        doc.ShowNamedView2("*Trimetric", -1); doc.ViewZoomtofit2();

        // STEP (save as copy, the open document keeps its name)
        string step = Path.Combine(cad, "CubeSat_3U_v5.step");
        sw.SetUserPreferenceIntegerValue(214 /* swStepAP */, 214);
        bool ok = doc.Extension.SaveAs(step, 0, 1 /* silent */ | 2 /* copy */, null, ref e, ref w);
        Console.WriteLine("STEP " + ok + " err " + e + " -> " + step + " (" + (File.Exists(step) ? new FileInfo(step).Length : 0) + " bytes)");

        // Pack and Go (flattened into one folder, names unchanged)
        string pg = Path.Combine(cad, "solidworks_v5");
        Directory.CreateDirectory(pg);
        var pgo = doc.Extension.GetPackAndGo();
        pgo.IncludeDrawings = false;
        pgo.IncludeSimulationResults = false;
        pgo.IncludeToolboxComponents = true;
        pgo.FlattenToSingleFolder = true;
        pgo.SetSaveToName(true, pg);
        object names; pgo.GetDocumentNames(out names);
        Console.WriteLine("pack and go: " + ((object[])names).Length + " documents");
        var st = (int[])doc.Extension.SavePackAndGo(pgo);
        int bad = 0; foreach (int s in st) if (s != 0) bad++;
        Console.WriteLine("pack and go statuses: " + st.Length + " files, non-zero " + bad);
        sw.CloseAllDocuments(true);
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
