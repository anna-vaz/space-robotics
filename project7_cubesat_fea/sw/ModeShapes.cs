// Reads the saved modal results (no solve) and prints, for the first modes, where the largest
// mode-shape amplitude is and which component that node belongs to (nearest-component by bounding box).
using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class ModeShapes
{
    public static void Run(string outDir)
    {
        var sw = SwSim.Connect(Console.Out);
        var doc = (ModelDoc2)sw.ActiveDoc;
        var cw = SwSim.Simulation(sw, Console.Out);
        var sm = cw.ActiveDoc.StudyManager;
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();
        string want = System.Environment.GetEnvironmentVariable("MS_STUDY") ?? "Modal_cantilever_v2_h4";
        int nm = int.Parse(System.Environment.GetEnvironmentVariable("MS_N") ?? "12");
        for (int i = 0; i < sm.StudyCount; i++)
        {
            var st = sm.GetStudy(i);
            if (st == null || st.Name != want) continue;
            var res = st.Results; int err;
            if (res == null) { Console.WriteLine("no results"); return; }
            for (int m = 1; m <= nm; m++)
            {
                var d = ((Array)res.GetMinMaxDisplacement(3, m, null, 0, out err)).Cast<object>().Select(Convert.ToDouble).ToArray();
                double x, y, z; st.Mesh.GetNodeLocation((int)d[2], out x, out y, out z);
                string hit = "?";
                foreach (var c in comps)
                {
                    var b = c.GetBox(false, false) as double[];
                    if (b != null && x >= b[0] - 1e-4 && x <= b[3] + 1e-4 && y >= b[1] - 1e-4 && y <= b[4] + 1e-4 && z >= b[2] - 1e-4 && z <= b[5] + 1e-4)
                    { if (hit == "?") hit = c.Name2; else hit += "|" + c.Name2; }
                }
                Console.WriteLine(string.Format("mode {0,2}: max amplitude at ({1:F1}, {2:F1}, {3:F1}) mm  in {4}", m, x * 1e3, y * 1e3, z * 1e3, hit));
            }
        }
    }
}
