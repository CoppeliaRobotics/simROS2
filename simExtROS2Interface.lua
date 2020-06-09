local simROS2 ={}

--@fun importInterface
--@arg name string the name of the interface to import, e.g.: geometry_msgs/msg/Vector3
function simROS2.importInterface(name)
    local name_comp={}
    for part in string.gmatch(name, "[^/]+") do
        table.insert(name_comp, part)
    end
    if #name_comp ~= 3 then
        return nil
    end
    local if_def=simROS2.getInterfaceConstants(name)
    if_def.__name=name
    if_def.__new=function() return simROS2.createInterface(name) end
    if _G[name_comp[1]] == nil then _G[name_comp[1]]={} end
    if _G[name_comp[1]][name_comp[2]] == nil then _G[name_comp[1]][name_comp[2]]={} end
    _G[name_comp[1]][name_comp[2]][name_comp[3]]=if_def
end

return simROS2
