#include <callbacks.h>
#include <simLib.h>
#include <stubs.h>
#include <cstring>

#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py for subinterface_name, subinterface in interface.subinterfaces.items():
void write__`subinterface.cpp_type_normalized`(const `subinterface.cpp_type`& msg, int stack, const WriteOptions *opt)
{
    try
    {
        simPushTableOntoStackE(stack);
#py for field in subinterface.fields:
#py if field.type.is_array:
#py if field.type.is_primitive_type() and field.type.type in fast_write_types:
        try
        {
            // write field '`field.name`' (using fast specialized function)
            simPushStringOntoStackE(stack, "`field.name`", 0);
            simPush`fast_write_types[field.type.type]`TableOntoStackE(stack, &(msg.`field.name`[0]), msg.`field.name`.size());
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py elif field.type.is_primitive_type() and field.type.type == 'uint8':
        try
        {
            // write field '`field.name`' (using fast specialized function)
            simPushStringOntoStackE(stack, "`field.name`", 0);
            if(opt && opt->uint8array_as_string)
                simPushStringOntoStackE(stack, (simChar*)&(msg.`field.name`[0]), msg.`field.name`.size());
            else
                simPushUInt8TableOntoStackE(stack, &(msg.`field.name`[0]), msg.`field.name`.size());
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py else:
        try
        {
            // write field '`field.name`'
            simPushStringOntoStackE(stack, "`field.name`", 0);
            simPushTableOntoStackE(stack);
            for(int i = 0; i < msg.`field.name`.size(); i++)
            {
                write__int32(i + 1, stack, opt);
                write__`field.cpp_type_normalized`(msg.`field.name`[i], stack, opt);
                simInsertDataIntoStackTableE(stack);
            }
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py endif
#py else:
        try
        {
            // write field '`field.name`'
            simPushStringOntoStackE(stack, "`field.name`", 0);
            write__`field.cpp_type_normalized`(msg.`field.name`, stack, opt);
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py endif
#py endfor
    }
    catch(exception& ex)
    {
        std::string msg = "write__`subinterface.cpp_type_normalized`: ";
        msg += ex.what();
        throw exception(msg);
    }
}

void read__`subinterface.cpp_type_normalized`(int stack, `subinterface.cpp_type` *msg, const ReadOptions *opt)
{
    try
    {
        int r = simGetStackTableInfoE(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw exception("expected a table");

        int oldsz = simGetStackSizeE(stack);
        simUnfoldStackTableE(stack);
        int numItems = (simGetStackSizeE(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            simMoveStackItemToTopE(stack, oldsz - 1); // move key to top
            if((str = simGetStackStringValueE(stack, &strSz)) != NULL && strSz > 0)
            {
                simPopStackItemE(stack, 1);

                simMoveStackItemToTopE(stack, oldsz - 1); // move value to top

                if(0) {}
#py for field in subinterface.fields:
#py if field.type.is_array:
#py if field.type.is_primitive_type() and field.type.type in fast_write_types:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`' (using fast specialized function)
                        int sz = simGetStackTableInfoE(stack, 0);
                        if(sz < 0)
                            throw exception("expected array");
                        if(simGetStackTableInfoE(stack, 2) != 1)
                            throw exception("fast_write_type reader exception #1");
#py if field.type.array_size:
                        // field has fixed size -> no need to reserve space into vector
#py else:
                        msg->`field.name`.resize(sz);
#py endif
                        simGetStack`fast_write_types[field.type.type]`TableE(stack, &(msg->`field.name`[0]), sz);
                        simPopStackItemE(stack, 1);
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py elif field.type.is_primitive_type() and field.type.type == 'uint8':
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field '`field.name`' (uint8[]) as string
                            simChar *str;
                            simInt sz;
                            if((str = simGetStackStringValueE(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
#py if field.type.array_size:
                                // field has fixed size -> no need to reserve space into vector
#py else:
                                msg->`field.name`.resize(sz);
#py endif
                                std::memcpy(&(msg->`field.name`[0]), str, sz);
                                simReleaseBufferE(str);
                            }
                            else throw exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field '`field.name`' (using fast specialized function)
                            int sz = simGetStackTableInfoE(stack, 0);
                            if(sz < 0)
                                throw exception("expected uint8 array");
                            if(simGetStackTableInfoE(stack, 2) != 1)
                                throw exception("fast_write_type uint8[] reader exception #1");
#py if field.type.array_size:
                            // field has fixed size -> no need to reserve space into vector
#py else:
                            msg->`field.name`.resize(sz);
#py endif
                            simGetStackUInt8TableE(stack, &(msg->`field.name`[0]), sz);
                            simPopStackItemE(stack, 1);
			}
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py else:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`'
                        if(simGetStackTableInfoE(stack, 0) < 0)
                            throw exception("expected array");
                        int oldsz1 = simGetStackSizeE(stack);
                        simUnfoldStackTableE(stack);
                        int numItems1 = (simGetStackSizeE(stack) - oldsz1 + 1) / 2;
                        for(int i = 0; i < numItems1; i++)
                        {
                            simMoveStackItemToTopE(stack, oldsz1 - 1); // move key to top
                            int j;
                            read__int32(stack, &j, opt);
                            simMoveStackItemToTopE(stack, oldsz1 - 1); // move value to top
                            `field.cpp_type` v;
                            read__`field.cpp_type_normalized`(stack, &v, opt);
#py if field.type.array_size:
                            msg->`field.name`[i] = (v);
#py else:
                            msg->`field.name`.push_back(v);
#py endif
                        }
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py endif
#py else:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`'
                        read__`field.cpp_type_normalized`(stack, &(msg->`field.name`), opt);
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py endif
#py endfor
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw exception(msg);
                }

                simReleaseBuffer(str);
            }
            else
            {
                throw exception("malformed table (bad key type)");
            }

            numItems = (simGetStackSizeE(stack) - oldsz + 1) / 2;
        }
    }
    catch(exception& ex)
    {
        std::string msg = "read__`subinterface.cpp_type_normalized`: ";
        msg += ex.what();
        throw exception(msg);
    }
}

#py endfor
#py endfor

#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
void ros_callback__`interface.cpp_type_normalized`(const `interface.cpp_type`::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = simCreateStackE();
        write__`interface.cpp_type_normalized`(*msg, stack, &(proxy->wr_opt));
        simCallScriptFunctionExE(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        simReleaseStackE(stack);
        stack = -1;
    }
    catch(exception& ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_callback__`interface.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(proxy->topicCallback.name.c_str(), msg.c_str());
    }
}

#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
bool ros_srv_callback__`interface.cpp_type_normalized`(const std::shared_ptr<rmw_request_id_t> request_header, const `interface.request.cpp_type`::SharedPtr req, `interface.response.cpp_type`::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = simCreateStackE();
        write__`interface.request.cpp_type_normalized`(*req, stack, &(proxy->wr_opt));
        simCallScriptFunctionExE(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__`interface.response.cpp_type_normalized`(stack, res.get(), &(proxy->rd_opt));
        simReleaseStackE(stack);
        stack = -1;
        return true;
    }
    catch(exception& ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__`interface.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(proxy->serviceCallback.name.c_str(), msg.c_str());
        return false;
    }
}

#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
// action server code

#py endif
#py endfor
