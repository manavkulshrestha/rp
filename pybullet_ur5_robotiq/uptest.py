def upf_test(mesh):
    c1 = mesh.copy()

    sc1 = trimesh.Scene([c1])

    tf_idx = top_faces_idx(c1)
    tf_f = c1.faces[tf_idx]
    
    tf_f = np.hstack([tf_f, tf_f[:,0].reshape(-1,1)])
    
    p1 = get_path(tf_f[1], c1)
    sc1.add_geometry([p1])
    
    return sc1

upf_test(cube).show()