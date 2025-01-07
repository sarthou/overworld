import bpy
import sys 
import pip
pip.main(['install','pyaml'])
import yaml
import os
import copy
#### Argument 
mesh_root_path = "/home/avigne/Projets/supervision/catkin_ws/src/restaurant_resources/models/" #chemin recepteur des exports
root = bpy.data.scenes["Adream"].collection
root_visible = bpy.context.view_layer.layer_collection  
#### script
 
extensions_types = {"OBJ" , "STL"}

def createFolderForExtensions(path_before , path_after):
    for extension in extensions_types:
        if not os.path.exists(path_before + extension + path_after):
            os.makedirs(path_before + extension + path_after)
    
def exportOneObject(obj, export_path_before , export_path_after):
    obj.select_set(True)
    for obj_child in obj.children:
        obj_child.select_set(True)
    if obj.animation_data :
        bpy.ops.object.duplicate()
        bpy.ops.anim.keyframe_clear_v3d()
    
    if "STL" in extensions_types:
        bpy.ops.wm.stl_export(filepath = export_path_before + "STL" + export_path_after + obj.name + ".stl" ,  check_existing=False, 
        export_selected_objects=True , forward_axis = 'X' , up_axis = 'Z')
    if "OBJ" in extensions_types:
        bpy.ops.wm.obj_export(filepath=export_path_before + "OBJ" + export_path_after + obj.name + ".obj" , check_existing=False,
        export_selected_objects=True, forward_axis='X',up_axis='Z')
                       
    if obj.animation_data :
        bpy.ops.object.delete()
    obj.select_set(False)
    
    for obj_child in obj.children:
        obj_child.select_set(False)

    
def createNewObjectNode(obj , mesh = ''):   
    node={}
    mesh=obj.name
    node_location={
            'position':{'x' : round(obj.location.x,4) ,
            'y' : round(obj.location.y,4) ,
            'z' : round(obj.location.z,4) }}
    
    node_rotation_euler={
            'orientation':{'x' : round(obj.rotation_euler.x,4) ,
            'y' : round(obj.rotation_euler.y,4) ,
            'z' : round(obj.rotation_euler.z,4) }}
           
    scalex=obj.dimensions.x / (bpy.data.objects[mesh].dimensions.x) 
    scaley=obj.dimensions.y / (bpy.data.objects[mesh].dimensions.y) 
    if (bpy.data.objects[mesh].dimensions.z) > 0:
        scalez=obj.dimensions.z / (bpy.data.objects[mesh].dimensions.z)
    else:
        scalez=1;       
                
    node_scale={
            'scale':{'x' : scalex ,
            'y' : scaley ,
            'z' : scalez }}
            
    if( (round(obj.location.x,4))!=0.0 or (round(obj.location.y,4))!=0.0 or (round(obj.location.z,4))!=0.0): 
        node.update(node_location)
    
    if(( (round(obj.rotation_euler.x,4))!=(0.0) or (round(obj.rotation_euler.y,4))!=(0.0)) or (round(obj.rotation_euler.z,4)!=(0.0))): 
        node.update(node_rotation_euler)
        
    if((scalex!=1.0) or (scaley!=1.0) or (scalez!=1.0)):
        node.update(node_scale)
              
    name = obj.name
    name = name.replace('.',"_")
    return (node , name)

def createYamlList(collection_dict,name):
    yaml_string = yaml.dump(collection_dict,default_flow_style=False )
    with open(mesh_root_path+name+".yaml" , 'w') as outfile :
        outfile.write(yaml_string)
            
def exportObjects():
    createFolderForExtensions(mesh_root_path , "/")
    collection_dict = {}
    for obj in root.objects:
        exportOneObject(obj , mesh_root_path , "/")
        obj_node , obj_name = createNewObjectNode(obj)
        collection_dict[obj_name] = obj_node
    for child in root.children:
        if child.name != "areas":
            exportObjectsInCollection(child , "/" , collection_dict , root_visible.children)      
    createYamlList(collection_dict,"export")
    
def exportObjectsInCollection(collection , collection_path, objects_dict , visible):
    local_collection_path = collection_path + collection.name + "/"
    createFolderForExtensions(mesh_root_path , local_collection_path)
    if visible[collection.name].is_visible :
        for obj in collection.objects:
                exportOneObject(obj , mesh_root_path , local_collection_path)  
                obj_node , obj_name = createNewObjectNode(obj)                    
                objects_dict[obj_name] = obj_node
        for child in collection.children:
            objects_dict[child.name] = {}
            exportObjectsInCollection(child , local_collection_path , objects_dict[child.name] , visible[collection.name].children)
            
def exportAreas():
    createFolderForExtensions(mesh_root_path , "/areas/")
    collection_dict = {}
    exportObjectsInCollection(root.children["areas"],"/",collection_dict,root_visible.children)
    createYamlList(collection_dict,"areas")
    
    
    
                
if __name__=="__main__":  
    for ob in bpy.context.selected_objects:
        ob.select_set(False)
    exportObjects()
    exportAreas()
    print('export end well')