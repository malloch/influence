
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <mapper/mapper.h>

struct _agentInfo
{
    char *influence_device_name;
    char *qualia_device_class;

    int proxy_influence_linked;
    int influence_qualia_linked;
    int qualia_proxy_linked;

    mapper_admin admin;
    mapper_device dev;
    mapper_monitor mon;
    mapper_db db;
} agentInfo;

mapper_signal sig_pos_in,
              sig_pos_out,
              sig_vel_in,
              sig_vel_out,
              sig_accel_in,
              sig_accel_out,
              sig_force;
mapper_timetag_t tt;

float mass = 1.0;
float gain = 0.0001;
float damping = 0.6;
float limit = 0.1;

#define WIDTH  500
#define HEIGHT 500

int numInstances = 1;
int *active;
int done = 0;

int compare_device_class(const char *device_name, const char *class_name)
{
    if (!device_name || !class_name)
        return 1;

    // find '.'
    int len = strchr(device_name, '.') - device_name;
    if (len != strlen(class_name))
        return 1;

    return strncmp(device_name, class_name, len);
}

void send_message(mapper_db_device dev)
{
    if (!dev)
        return;

    printf("PROVOKING QUALIA AGENT: %s\n", dev->name);
    /* To allow a multi-instance network with the single-instance Qualia
     * agent programs, we need the Qualia agents to initiate message-passing.
     * However Qualia agents will not generate an "action" (output) until
     * _after_ they have received an "observation" (input). This function
     * "cheats" and sends an out-of-band message to the observation input of
     * the Qualia agent to provoke a response and kick things off... */

    // Retrieve device's IP and port
    lo_type type;
    const lo_arg *val;
    const char *host;
    int port;

    int i=0, found=0;
    const char *key;
    // TODO: use mapper_db_device_property_lookup() instead
    while(!mapper_db_device_property_index(dev, i++, &key, &type, &val)
          && found < 2)
    {
        if (strcmp(key, "host")==0) {
            host = &val->s;
            found++;
        }
        else if (strcmp(key, "port")==0) {
            port = val->i32;
            found++;
        }
    }
    if (found < 2) {
        printf("Couldn't retrieve device host and port\n");
        return;
    }

    printf("Retrieved IP %s and port %i for device %s\n", host, port, dev->name);
    char urlstr[128];
    snprintf(urlstr, 128, "osc.udp://%s:%i", host, port);
    lo_address a = lo_address_new_from_url(urlstr);
    lo_send(a, "/observation", "ff", 0.f, 0.f);
    lo_address_free(a);
}

void provoke_qualia_agent(const char *name)
{
    /* TODO: we could check update time on local instances and only provoke
     * connected agents that haven't been in contact recently. */
    struct _agentInfo *info = &agentInfo;

    if (!info->db)
        return;

    if (name) {
        send_message(mapper_db_get_device_by_name(info->db, name));
    }
    else {
        mapper_db_device *pdev = mapper_db_get_all_devices(info->db);
        while (pdev) {
            if (compare_device_class((*pdev)->name, "/agent") == 0)
                send_message(*pdev);
            pdev = mapper_db_device_next(pdev);
        }
    }
}

void init_instance(int id)
{
    float init[] = {0, 0};

    // If force instance not active, initialize to zero
    if (!msig_instance_value(sig_force, id, 0))
        msig_update_instance(sig_force, id, &init, 1, MAPPER_NOW);

    // If acceleration input instance not active, initialize to zero
    if (!msig_instance_value(sig_accel_in, id, 0))
        msig_update_instance(sig_accel_in, id, &init, 1, MAPPER_NOW);

    // If velocity instance not active, initialize to zero
    if (!msig_instance_value(sig_vel_in, id, 0))
        msig_update_instance(sig_vel_in, id, &init, 1, MAPPER_NOW);

    // If position instance not active, initialize randomly
    if (!msig_instance_value(sig_pos_in, id, 0)) {
        init[0] = rand()%1000*0.002-1.0;
        init[1] = rand()%1000*0.002-1.0;
        msig_update_instance(sig_pos_in, id, &init, 1, MAPPER_NOW);
    }
    active[id] = 1;
}

void release_instance(int instance_id)
{
    msig_release_instance(sig_pos_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_pos_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_vel_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_vel_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_accel_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_accel_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_force, instance_id, MAPPER_NOW);
    active[instance_id] = 0;
}

void force_handler(mapper_signal msig,
                   mapper_db_signal props,
                   int instance_id,
                   void *value,
                   int count,
                   mapper_timetag_t *timetag)
{
    if (value && !active[instance_id]) {
        init_instance(instance_id);
    }
    else if (!value && active[instance_id]) {
        release_instance(instance_id);
        return;
    }

    float *force = (float *)value;

    // we stored a pointer to sig_accel_in in user_data
    mapper_signal accel_sig = (mapper_signal)props->user_data;
    float accel[2], *paccel;

    paccel = (float *)msig_instance_value(accel_sig, instance_id, 0);
    if (!paccel) {
        printf("error: force_handler cannot retrieve acceleration\n");
        return;
    }

    accel[0] = paccel[0] + force[0] / mass * gain;
    accel[1] = paccel[1] + force[1] / mass * gain;
    msig_update_instance(accel_sig, instance_id, &accel, 1, MAPPER_NOW);
}

void generic_handler(mapper_signal msig,
                     mapper_db_signal props,
                     int instance_id,
                     void *value,
                     int count,
                     mapper_timetag_t *timetag)
{
    if (value && !active[instance_id]) {
        init_instance(instance_id);
    }
    else if (!value && active[instance_id]) {
        release_instance(instance_id);
    }
}

void dev_db_callback(mapper_db_device record,
                     mapper_db_action_t action,
                     void *user)
{
    struct _agentInfo *info = (struct _agentInfo*)user;

    if (action == MDB_NEW) {
        printf("Received device %s\n", record->name);
        if (compare_device_class(record->name,
                                 info->qualia_device_class)==0) {

            // this link will default to correct scope
            mapper_monitor_link(info->mon, record->name,
                                mdev_name(info->dev), 0, 0);

            // link influence->qualia
            // this link needs to be scoped for qualia instances
            mapper_db_link_t props;
            props.num_scopes = 1;
            props.scope_names = &record->name;
            mapper_monitor_link(info->mon, info->influence_device_name,
                                record->name, &props,
                                LINK_NUM_SCOPES | LINK_SCOPE_NAMES);
        }
        else if (strcmp(record->name, info->influence_device_name)==0) {
            // Use scope "all" for this link to allow all instances
            mapper_db_link_t props;
            props.num_scopes = 1;
            char *name = "all";
            props.scope_names = &name;
            mapper_monitor_link(info->mon, mdev_name(info->dev), record->name,
                                &props, LINK_NUM_SCOPES | LINK_SCOPE_NAMES);

            // link influence to each qualia program
            mapper_db_device *dev =
                mapper_db_match_devices_by_name(info->db,
                                                info->qualia_device_class);
            while (dev) {
                lo_type type;
                const lo_arg *val;
                if (!mapper_db_device_property_lookup(*dev, "name", &type, &val)) {
                    const char *name = &val->s;
                    props.scope_names = (char **)&name;
                    mapper_monitor_link(info->mon, info->influence_device_name,
                                        &val->s, &props,
                                        LINK_NUM_SCOPES | LINK_SCOPE_NAMES);
                }
                dev = mapper_db_device_next(dev);
            }
        }
    }
    else if (action == MDB_REMOVE) {
        if (compare_device_class(record->name, info->qualia_device_class)==0) {
            mapper_monitor_unlink(info->mon, record->name,
                                  mdev_name(info->dev));
        }
        else if (strcmp(record->name, info->influence_device_name)==0) {
            mapper_monitor_unlink(info->mon, mdev_name(info->dev),
                                  record->name);
        }
    }
}

void link_db_callback(mapper_db_link record,
                      mapper_db_action_t action,
                      void *user)
{
    // if we see our links, send /connect messages
    char signame1[1024], signame2[1024];
    struct _agentInfo *info = (struct _agentInfo*)user;

    if ((strcmp(record->src_name, mdev_name(info->dev))==0) &&
        (strcmp(record->dest_name, info->influence_device_name)==0)) {
        if (action == MDB_NEW) {
            printf("Received link %s -> %s\n",
                   record->src_name, record->dest_name);
            // send proxy->influence connections
            sprintf(signame1, "%s/position", mdev_name(info->dev));
            sprintf(signame2, "%s/node/position", info->influence_device_name);
            mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);
            info->proxy_influence_linked++;
        }
        else if (action == MDB_REMOVE) {
            info->proxy_influence_linked--;
        }
    }
    else if ((strcmp(record->src_name, info->influence_device_name)==0) &&
        (compare_device_class(record->dest_name, info->qualia_device_class))==0) {
        if (action == MDB_NEW) {
            printf("Received link %s -> %s\n",
                   record->src_name, record->dest_name);
            // send influence->qualia connections
            sprintf(signame1, "%s/node/observation", info->influence_device_name);
            sprintf(signame2, "%s/observation", record->dest_name);
            mapper_db_connection_t props;
            props.send_as_instance = 1;
            mapper_monitor_connect(info->mon, signame1, signame2, &props,
                                   CONNECTION_SEND_AS_INSTANCE);
            info->influence_qualia_linked++;
            provoke_qualia_agent(record->dest_name);
        }
        else if (action == MDB_REMOVE) {
            info->influence_qualia_linked--;
        }
    }
    else if ((compare_device_class(record->src_name, info->qualia_device_class)==0) &&
             (strcmp(record->dest_name, mdev_name(info->dev))==0)) {
        if (action == MDB_NEW) {
            printf("Received link %s -> %s\n",
                   record->src_name, record->dest_name);
            // send qualia->proxy connections
            // we will override the default "sendAsInstance" property
            sprintf(signame1, "%s/action", record->src_name);
            sprintf(signame2, "%s/force", mdev_name(info->dev));
            mapper_db_connection_t props;
            props.send_as_instance = 1;
            mapper_monitor_connect(info->mon, signame1, signame2, &props,
                                   CONNECTION_SEND_AS_INSTANCE);
            info->qualia_proxy_linked++;
            provoke_qualia_agent(record->src_name);
        }
        else if (action == MDB_REMOVE) {
            info->qualia_proxy_linked--;
        }
    }
}

struct _agentInfo *agentInit()
{
    struct _agentInfo *info = &agentInfo;
    memset(info, 0, sizeof(struct _agentInfo));

    info->influence_device_name = strdup("/influence.1");
    info->qualia_device_class = strdup("/agent");

    active = (int *)calloc(1, sizeof(int)*numInstances);

    info->admin = mapper_admin_new(0, 0, 0);

    // add device
    info->dev = mdev_new("proxyAgent", 0, info->admin);
    while (!mdev_ready(info->dev)) {
        mdev_poll(info->dev, 100);
    }
    printf("ordinal: %d\n", mdev_ordinal(info->dev));
    fflush(stdout);

    // add monitor and monitor callbacks
    info->mon = mapper_monitor_new(info->admin, 0);
    info->db  = mapper_monitor_get_db(info->mon);
    mapper_db_add_device_callback(info->db, dev_db_callback, info);
    mapper_db_add_link_callback(info->db, link_db_callback, info);

    // add signals
    float mn=-1, mx=1;

    // Add acceleration input and output signals
    sig_accel_in = mdev_add_input(info->dev, "acceleration", 2, 'f', 0,
                                  &mn, &mx, generic_handler, 0);
    sig_accel_out = mdev_add_output(info->dev, "acceleration", 2, 'f', 0, &mn, &mx);

    // Release the default instances
    msig_release_instance(sig_accel_in, 0, MAPPER_NOW);
    msig_release_instance(sig_accel_out, 0, MAPPER_NOW);

    // Reserve more instances
    msig_reserve_instances(sig_accel_in, numInstances-1, 0, 0);
    msig_reserve_instances(sig_accel_out, numInstances-1, 0, 0);

    // Add force signals
    sig_force = mdev_add_input(info->dev, "force", 2, 'f', "N", &mn, &mx,
                               force_handler, sig_accel_in);
    msig_release_instance(sig_force, 0, MAPPER_NOW);
    msig_reserve_instances(sig_force, numInstances-1, 0, 0);

    // Add velocity signals
    sig_vel_in = mdev_add_input(info->dev, "velocity", 2, 'f', "m/s",
                                &mn, &mx, generic_handler, 0);
    sig_vel_out = mdev_add_output(info->dev, "velocity", 2, 'f', "m/s", &mn, &mx);

    msig_release_instance(sig_vel_in, 0, MAPPER_NOW);
    msig_release_instance(sig_vel_out, 0, MAPPER_NOW);

    msig_reserve_instances(sig_vel_in, numInstances-1, 0, 0);
    msig_reserve_instances(sig_vel_out, numInstances-1, 0, 0);

    // add position signals
    sig_pos_in = mdev_add_input(info->dev, "position", 2, 'f', 0,
                                &mn, &mx, generic_handler, 0);
    sig_pos_out = mdev_add_output(info->dev, "position", 2, 'f', 0, &mn, &mx);

    msig_release_instance(sig_pos_in, 0, MAPPER_NOW);
    msig_release_instance(sig_pos_out, 0, MAPPER_NOW);

    msig_reserve_instances(sig_pos_in, numInstances-1, 0, 0);
    msig_reserve_instances(sig_pos_out, numInstances-1, 0, 0);

    return info;
}

void agentLogout()
{
    printf("Cleaning up...\n");
    struct _agentInfo *info = &agentInfo;

    int i;
    mdev_now(info->dev, &tt);
    mdev_start_queue(info->dev, tt);
    for (i=0; i<numInstances; i++) {
        msig_release_instance(sig_pos_out, i, tt);
        msig_release_instance(sig_vel_out, i, tt);
        msig_release_instance(sig_accel_out, i, tt);
    }
    mdev_send_queue(info->dev, tt);

    // TODO: unlink devices
    if (info->influence_qualia_linked) {
        mapper_monitor_unlink(info->mon,
                              info->influence_device_name,
                              info->qualia_device_class);
    }
    if (info->influence_device_name) {
        mapper_monitor_unlink(info->mon,
                              mdev_name(info->dev),
                              info->influence_device_name);
        free(info->influence_device_name);
    }
    if (info->qualia_device_class) {
        free(info->qualia_device_class);
    }
    if (info->dev)
        mdev_free(info->dev);
    if (info->db) {
        mapper_db_remove_device_callback(info->db, dev_db_callback, info);
        mapper_db_remove_link_callback(info->db, link_db_callback, info);
    }
    if (info->mon)
        mapper_monitor_free(info->mon);
    if (info->admin) {
        mapper_admin_free(info->admin);
    }
    memset(info, 0, sizeof(struct _agentInfo));
    if (active)
        free(active);
}

void ctrlc(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    int i, j, id, counter=0;
    if (argc > 1)
        numInstances = atoi(argv[1]);

    signal(SIGINT, ctrlc);

    srand(100);

    struct _agentInfo *info = agentInit();
    if (!info->dev)
        goto done;

    float *paccel, *pvel, *ppos;
    float accel[2], vel[2], pos[2];

    while (!mdev_ready(info->dev)) {
        mapper_monitor_poll(info->mon, 0);
        mdev_poll(info->dev, 10);
    }

    while (!done) {
        mapper_monitor_poll(info->mon, 0);
        mdev_poll(info->dev, 20);
        if (counter++ > 100) {
            provoke_qualia_agent(0);
            counter = 0;
        }

        mdev_now(info->dev, &tt);
        mdev_start_queue(info->dev, tt);
        int num = msig_num_active_instances(sig_force);
        for (i = 0; i < num; i++) {
            id = msig_active_instance_id(sig_force, i);

            paccel = (float *)msig_instance_value(sig_accel_in, id, 0);
            pvel = (float *)msig_instance_value(sig_vel_in, id, 0);
            if (!pvel) {
                printf("couldn't retrieve vel value for instance %i\n", id);
                continue;
            }
            if (!paccel) {
                printf("couldn't retrieve accel value for instance %i\n", id);
                continue;
            }

            ppos = (float *)msig_instance_value(sig_pos_in, id, 0);
            if (!ppos) {
                printf("couldn't retrieve pos value for instance %i\n", id);
                continue;
            }

            for (j = 0; j < 2; j++) {
                accel[j] = paccel[j] * 0.9;
                vel[j] = pvel[j] * 0.9 + accel[j];
                pos[j] = ppos[j] + vel[j];

                if (pos[j] < -1) {
                    pos[j] = -1;
                    vel[j] *= -0.95;
                }
                if (pos[j] >= 1) {
                    pos[j] = 1;
                    vel[j] *= -0.95;
                }

                accel[j] = 0;
            }

            msig_update_instance(sig_accel_in, id, &accel, 1, tt);
            msig_update_instance(sig_accel_out, id, &accel, 1, tt);
            msig_update_instance(sig_vel_in, id, &vel, 1, tt);
            msig_update_instance(sig_vel_out, id, &vel, 1, tt);
            msig_update_instance(sig_pos_in, id, &pos, 1, tt);
            msig_update_instance(sig_pos_out, id, &pos, 1, tt);
        }
        mdev_send_queue(info->dev, tt);
    }

done:
    agentLogout();
    return 0;
}
