// Shared helpers for driving SOLIDWORKS + SOLIDWORKS Simulation from PowerShell.
// Compiled at run time with Add-Type (C# 5, .NET Framework), early-bound through
// the official interop assemblies so out-parameters and typed calls work.
using System;
using System.IO;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class SwSim
{
    public const string AddinDll =
        @"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\Simulation\cosworks.dll";

    public static SldWorks Connect(TextWriter log)
    {
        // Attaches to a running SOLIDWORKS or starts one.
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        log.WriteLine("SOLIDWORKS revision: " + sw.RevisionNumber());
        return sw;
    }

    public static CosmosWorks Simulation(SldWorks sw, TextWriter log)
    {
        int st = sw.LoadAddIn(AddinDll);   // 0 = loaded now, 2 = already loaded
        log.WriteLine("LoadAddIn(Simulation) status: " + st);
        var cb = (CwAddincallback)sw.GetAddInObject("SldWorks.Simulation");
        if (cb == null) throw new Exception("Simulation add-in object not available");
        CosmosWorks cw = cb.CosmosWorks;
        log.WriteLine("Simulation version: " + cw.VersionNumber);
        return cw;
    }

    // New part with a rectangular block a x b in the Front plane, extruded L (metres).
    public static ModelDoc2 NewBlockPart(SldWorks sw, double a, double b, double L, string material)
    {
        string tmpl = sw.GetUserPreferenceStringValue(8); // swDefaultTemplatePart
        var doc = (ModelDoc2)sw.NewDocument(tmpl, 0, 0, 0);
        doc.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, false, 0, null, 0);
        doc.SketchManager.InsertSketch(true);
        doc.SketchManager.CreateCornerRectangle(0, 0, 0, a, b, 0);
        doc.SketchManager.InsertSketch(true);
        doc.FeatureManager.FeatureExtrusion2(true, false, false, 0, 0, L, 0,
            false, false, false, false, 0, 0, false, false, false, false,
            true, false, true, 0, 0, false);
        ((PartDoc)doc).SetMaterialPropertyName2("", "SOLIDWORKS Materials", material);
        doc.ClearSelection2(true);
        return doc;
    }

    // Planar faces of the (single) body whose bounding box lies on coordinate `axis` = value.
    public static object[] FacesOnPlane(ModelDoc2 doc, int axis, double value, double tol)
    {
        var faces = new List<object>();
        object[] bodies = (object[])((PartDoc)doc).GetBodies2(0, true);
        foreach (Body2 body in bodies)
            foreach (Face2 f in (object[])body.GetFaces())
            {
                double[] box = (double[])f.GetBox();
                if (Math.Abs(box[axis] - value) < tol && Math.Abs(box[axis + 3] - value) < tol)
                    faces.Add(f);
            }
        return faces.ToArray();
    }

    public static string Join(object arr)
    {
        if (arr == null) return "null";
        var parts = new List<string>();
        foreach (var o in (Array)arr) parts.Add(Convert.ToString(o, System.Globalization.CultureInfo.InvariantCulture));
        return string.Join(", ", parts.ToArray());
    }
}
