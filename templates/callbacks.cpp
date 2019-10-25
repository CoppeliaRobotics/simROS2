#include <callbacks.h>
#include <simLib.h>
#include <stubs.h>
#include <cstring>

#py import parse_messages_and_services as p
#py TypeSpec = p.TypeSpec
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgssrvs.items():
void write__`info.typespec.normalized()`(const `info.typespec.cpp_type()`& msg, int stack, const WriteOptions *opt)
{
    try
    {
        simPushTableOntoStackE(stack);
#py for n, t in info.fields.items():
#py if t.array:
#py if t.builtin and t.mtype in TypeSpec.fast_write_types:
        try
        {
            // write field '`n`' (using fast specialized function)
            simPushStringOntoStackE(stack, "`n`", 0);
            simPush`TypeSpec.fast_write_types[t.mtype]`TableOntoStackE(stack, &(msg.`n`[0]), msg.`n`.size());
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py elif t.builtin and t.mtype == 'uint8':
        try
        {
            // write field '`n`' (using fast specialized function)
            simPushStringOntoStackE(stack, "`n`", 0);
            if(opt && opt->uint8array_as_string)
                simPushStringOntoStackE(stack, (simChar*)&(msg.`n`[0]), msg.`n`.size());
            else
                simPushUInt8TableOntoStackE(stack, &(msg.`n`[0]), msg.`n`.size());
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py else:
        try
        {
            // write field '`n`'
            simPushStringOntoStackE(stack, "`n`", 0);
            simPushTableOntoStackE(stack);
            for(int i = 0; i < msg.`n`.size(); i++)
            {
                write__int32(i + 1, stack, opt);
                write__`t.normalized()`(msg.`n`[i], stack, opt);
                simInsertDataIntoStackTableE(stack);
            }
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py endif
#py else:
        try
        {
            // write field '`n`'
            simPushStringOntoStackE(stack, "`n`", 0);
            write__`t.normalized()`(msg.`n`, stack, opt);
            simInsertDataIntoStackTableE(stack);
        }
        catch(exception& ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw exception(msg);
        }
#py endif
#py endfor
    }
    catch(exception& ex)
    {
        std::string msg = "write__`info.typespec.normalized()`: ";
        msg += ex.what();
        throw exception(msg);
    }
}

void read__`info.typespec.normalized()`(int stack, `info.typespec.cpp_type()` *msg, const ReadOptions *opt)
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
#py for n, t in info.fields.items():
#py if t.array:
#py if t.builtin and t.mtype in TypeSpec.fast_write_types:
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`' (using fast specialized function)
                        int sz = simGetStackTableInfoE(stack, 0);
                        if(sz < 0)
                            throw exception("expected array");
                        if(simGetStackTableInfoE(stack, 2) != 1)
                            throw exception("fast_write_type reader exception #1");
#py if t.array_size:
                        // field has fixed size -> no need to reserve space into vector
#py else:
                        msg->`n`.resize(sz);
#py endif
                        simGetStack`TypeSpec.fast_write_types[t.mtype]`TableE(stack, &(msg->`n`[0]), sz);
                        simPopStackItemE(stack, 1);
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py elif t.builtin and t.mtype == 'uint8':
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field '`n`' (uint8[]) as string
                            simChar *str;
                            simInt sz;
                            if((str = simGetStackStringValueE(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
#py if t.array_size:
                                // field has fixed size -> no need to reserve space into vector
#py else:
                                msg->`n`.resize(sz);
#py endif
                                std::memcpy(&(msg->`n`[0]), str, sz);
                                simReleaseBufferE(str);
                            }
                            else throw exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field '`n`' (using fast specialized function)
                            int sz = simGetStackTableInfoE(stack, 0);
                            if(sz < 0)
                                throw exception("expected uint8 array");
                            if(simGetStackTableInfoE(stack, 2) != 1)
                                throw exception("fast_write_type uint8[] reader exception #1");
#py if t.array_size:
                            // field has fixed size -> no need to reserve space into vector
#py else:
                            msg->`n`.resize(sz);
#py endif
                            simGetStackUInt8TableE(stack, &(msg->`n`[0]), sz);
                            simPopStackItemE(stack, 1);
			}
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py else: # array not fast func
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`'
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
                            `t.cpp_type()` v;
                            read__`t.normalized()`(stack, &v, opt);
#py if t.array_size:
                            msg->`n`[i] = (v);
#py else:
                            msg->`n`.push_back(v);
#py endif
                        }
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw exception(msg);
                    }
                }
#py endif
#py else: # not array
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`'
                        read__`t.normalized()`(stack, &(msg->`n`), opt);
                    }
                    catch(exception& ex)
                    {
                        std::string msg = "field `n`: ";
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
        std::string msg = "read__`info.typespec.normalized()`: ";
        msg += ex.what();
        throw exception(msg);
    }
}

#py endfor
#py for msg, info in msgs.items():
void ros_callback__`info.typespec.normalized()`(const `info.typespec.cpp_type()`::SharedPtr msg, SubscriberProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = simCreateStackE();
        write__`info.typespec.normalized()`(*msg, stack, &(proxy->wr_opt));
        simCallScriptFunctionExE(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        simReleaseStackE(stack);
        stack = -1;
    }
    catch(exception& ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_callback__`info.typespec.normalized()`: ";
        msg += ex.what();
        simSetLastError(proxy->topicCallback.name.c_str(), msg.c_str());
    }
}

#py endfor
#py for srv, info in srvs.items():
bool ros_srv_callback__`info.typespec.normalized()`(const std::shared_ptr<rmw_request_id_t> request_header, const `info.typespec.cpp_type()`::Request::SharedPtr req, `info.typespec.cpp_type()`::Response::SharedPtr res, ServiceServerProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = simCreateStackE();
        write__`info.typespec.normalized()`Request(*req, stack, &(proxy->wr_opt));
        simCallScriptFunctionExE(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__`info.typespec.normalized()`Response(stack, res.get(), &(proxy->rd_opt));
        simReleaseStackE(stack);
        stack = -1;
        return true;
    }
    catch(exception& ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__`info.typespec.normalized()`: ";
        msg += ex.what();
        simSetLastError(proxy->serviceCallback.name.c_str(), msg.c_str());
        return false;
    }
}

#py endfor
