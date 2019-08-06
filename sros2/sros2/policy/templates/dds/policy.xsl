<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
  extension-element-prefixes="func"
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema"
  xmlns:func="http://exslt.org/functions"
  xmlns:local="http://example.com/nss/dummy"
  xmlns:ros="http://www.ros.org/"
  xmlns:ext="http://exslt.org/common" exclude-result-prefixes="ext ros xs xsl local">
<xsl:output omit-xml-declaration="yes" indent="yes"/>
<xsl:strip-space elements="*"/>

<ros:action>
  <mappings>
    <mapping type="reply"    prefix="rr"  suffix="/_action/cancel_goalReply"/>
    <mapping type="reply"    prefix="rr"  suffix="/_action/get_resultReply"/>
    <mapping type="reply"    prefix="rr"  suffix="/_action/send_goalReply"/>
    <mapping type="request"  prefix="rq"  suffix="/_action/cancel_goalRequest"/>
    <mapping type="request"  prefix="rq"  suffix="/_action/get_resultRequest"/>
    <mapping type="request"  prefix="rq"  suffix="/_action/send_goalRequest"/>
    <mapping type="feedback" prefix="rt"  suffix="/_action/feedback"/>
    <mapping type="status"   prefix="rt"  suffix="/_action/status"/>
  </mappings>
  <permissions>
    <permission kind="publish"   type="feedback" rule="execute"/>
    <permission kind="subscribe" type="feedback" rule="call"/>
    <permission kind="publish"   type="reply"    rule="execute"/>
    <permission kind="subscribe" type="reply"    rule="call"/>
    <permission kind="publish"   type="request"  rule="call"/>
    <permission kind="subscribe" type="request"  rule="execute"/>
    <permission kind="publish"   type="status"   rule="execute"/>
    <permission kind="subscribe" type="status"   rule="call"/>
  </permissions>
</ros:action>
<ros:service>
  <mappings>
    <mapping type="reply"    prefix="rr"  suffix="Reply"/>
    <mapping type="request"  prefix="rq"  suffix="Request"/>
  </mappings>
  <permissions>
    <permission kind="publish"   type="reply"    rule="reply"/>
    <permission kind="subscribe" type="reply"    rule="request"/>
    <permission kind="publish"   type="request"  rule="request"/>
    <permission kind="subscribe" type="request"  rule="reply"/>
  </permissions>
</ros:service>
<ros:topic>
  <mappings>
    <mapping type="topic"    prefix="rt"  suffix=""/>
  </mappings>
  <permissions>
    <permission kind="publish"   type="topic"    rule="publish"/>
    <permission kind="subscribe" type="topic"    rule="subscribe"/>
  </permissions>
</ros:topic>


<xsl:template match="@* | node()" mode="deduplicate">
  <xsl:copy>
    <xsl:apply-templates select="@* | node()" mode="deduplicate"/>
  </xsl:copy>
</xsl:template>


<xsl:key name="actions_equal" match="profile/actions" use=" concat(
    generate-id(..), '|',
    @call, '|',
    @execute, '|',
    .)"/>
<xsl:template mode="deduplicate" match="
  profile/actions[not(generate-id() =
  generate-id(key('actions_equal', concat(
    generate-id(..), '|',
    @call, '|',
    @execute, '|',
    .))[1]))]"/>


<xsl:key name="services_equal" match="profile/services" use=" concat(
    generate-id(..), '|',
    @reply, '|',
    @request, '|',
    .)"/>
<xsl:template mode="deduplicate" match="
  profile/services[not(generate-id() =
  generate-id(key('services_equal', concat(
    generate-id(..), '|',
    @reply, '|',
    @request, '|',
    .))[1]))]"/>


<xsl:key name="topics_equal" match="profile/topics" use=" concat(
    generate-id(..), '|',
    @publish, '|',
    @subscribe, '|',
    .)"/>
<xsl:template mode="deduplicate" match="
  profile/topics[not(generate-id() =
  generate-id(key('topics_equal', concat(
    generate-id(..), '|',
    @publish, '|',
    @subscribe, '|',
    .))[1]))]"/>


<xsl:template match="@* | node()" mode="compress">
  <xsl:copy>
    <xsl:apply-templates select="@* | node()" mode="compress"/>
  </xsl:copy>
</xsl:template>

<xsl:key name="actions_compatible" match="actions" use="concat(
  @call, '|',
  @execute)"/>
<xsl:key name="services_compatible" match="services" use="concat(
  @reply, '|',
  @request)"/>
<xsl:key name="topics_compatible" match="topics" use="concat(
  @puplish, '|',
  @subscribe)"/>

<xsl:template mode="compress" match="profile">
  <xsl:copy>
    <xsl:copy-of select="@*" />
    <xsl:apply-templates mode="sibling-recurse" select="
      actions[generate-id(.) = generate-id(key('actions_compatible', concat(
      @call, '|',
      @execute)))]"/>
    <xsl:apply-templates mode="sibling-recurse" select="
      services[generate-id(.) = generate-id(key('services_compatible', concat(
      @reply, '|',
      @request)))]"/>
    <xsl:apply-templates mode="sibling-recurse" select="
      topics[generate-id(.) = generate-id(key('topics_compatible', concat(
      @puplish, '|',
      @subscribe)))]"/>
  </xsl:copy>
</xsl:template>

<xsl:template match="actions" mode="sibling-recurse">
  <xsl:copy>
    <xsl:apply-templates mode="compress" select="node() | @*" />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@call = current()/@call]/node() " />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@execute = current()/@execute]/node() " />
  </xsl:copy>
</xsl:template>
<xsl:template match="services" mode="sibling-recurse">
  <xsl:copy>
    <xsl:apply-templates mode="compress" select="node() | @*" />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@reply = current()/@reply]/node() " />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@request = current()/@request]/node() " />
  </xsl:copy>
</xsl:template>
<xsl:template match="topics" mode="sibling-recurse">
  <xsl:copy>
    <xsl:apply-templates mode="compress" select="node() | @*" />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@puplish = current()/@puplish]/node() " />
    <xsl:apply-templates mode="compress" select="
      following-sibling::services[@subscribe = current()/@subscribe]/node() " />
  </xsl:copy>
</xsl:template>


<xsl:variable name="policy_version" select="'0.1.0'"/>
<xsl:template match="/polciy/profiles">
  <xsl:variable name="policy">
    <policy version="{$policy_version}">
      <profiles>
        <xsl:for-each select="profile">
          <xsl:sort select="profile"/>
          <xsl:variable name="profile">
            <profile ns="{@ns}" node="{@node}">
              <xsl:variable name="_ns">
                <xsl:call-template name="DelimitNamespace">
                  <xsl:with-param name="ns" select="@ns"/>
                </xsl:call-template>
              </xsl:variable>
              <xsl:variable name="_fqn">
                <xsl:value-of select="concat($_ns, @node)"/>
              </xsl:variable>
              <xsl:if test="./*[@* = 'DENY']">
                <xsl:for-each select="./*[@* = 'DENY']">
                  <xsl:call-template name="TranslatePermissions">
                  <xsl:with-param name="qualifier" select="'DENY'"/>
                  </xsl:call-template>
                </xsl:for-each>
              </xsl:if>
              <xsl:if test="./*[@* = 'ALLOW']">
                <xsl:for-each select="./*[@* = 'ALLOW']">
                  <xsl:call-template name="TranslatePermissions">
                  <xsl:with-param name="qualifier" select="'ALLOW'"/>
                  </xsl:call-template>
                </xsl:for-each>
              </xsl:if>
            </profile>
          </xsl:variable>

          <xsl:variable name="profile_deduplicated">
            <xsl:apply-templates mode="deduplicate"
              select="ext:node-set($profile)"/>
          </xsl:variable>

          <xsl:variable name="profile_namespaced">
            <xsl:apply-templates mode="namespace"
              select="ext:node-set($profile_deduplicated)"/>
          </xsl:variable>

          <xsl:apply-templates mode="compress"
            select="ext:node-set($profile_namespaced)"/>

        </xsl:for-each>
      </profiles>
    </policy>
  </xsl:variable>

 <xsl:apply-templates mode="sort"
   select="ext:node-set($policy)"/>
</xsl:template>

<xsl:template name="TranslatePermissions">
  <xsl:param name="qualifier"/>

  <xsl:if test="@publish = $qualifier">
    <xsl:call-template name="dds_topics">
      <xsl:with-param name="kind" select="'publish'"/>
      <xsl:with-param name="qualifier" select="$qualifier"/>
    </xsl:call-template>
  </xsl:if>
  <xsl:if test="@subscribe = $qualifier">
    <xsl:call-template name="dds_topics">
      <xsl:with-param name="kind" select="'subscribe'"/>
      <xsl:with-param name="qualifier" select="$qualifier"/>
    </xsl:call-template>
  </xsl:if>
</xsl:template>


<xsl:template name="dds_topics">
  <xsl:param name="kind"/>
  <xsl:param name="qualifier"/>

  <xsl:for-each select="dds_topic">
    <xsl:variable name="dds_topic_" select="text()" />
    <xsl:variable name="prefix" select="substring-before($dds_topic_, '/')" />
    <xsl:variable name="_dds_topic" select="substring-after($dds_topic_, $prefix)" />

    <xsl:variable name="actions_"
      select="document('')/xsl:stylesheet/
        ros:action/mappings/mapping[@prefix=$prefix]" />
    <xsl:variable name="services_"
      select="document('')/xsl:stylesheet/
        ros:service/mappings/mapping[@prefix=$prefix]" />
    <xsl:variable name="topics_"
      select="document('')/xsl:stylesheet/
        ros:topic/mappings/mapping[@prefix=$prefix]" />

      <xsl:variable name="action_match">      
        <xsl:for-each select="$actions_">
          <xsl:variable name="suffix" select="
            substring($_dds_topic,
            string-length($_dds_topic) - string-length(@suffix) +1)" />
          <xsl:if test="$suffix = @suffix">
            <xsl:variable name="object" select="substring($_dds_topic, 1, 
                  string-length($_dds_topic) - string-length($suffix))" />
            <xsl:call-template name="actions">
              <xsl:with-param name="action" select="$object"/>
              <xsl:with-param name="kind" select="$kind"/>
              <xsl:with-param name="type" select="
              $actions_[@prefix=$prefix and @suffix=$suffix]/@type"/>
              <xsl:with-param name="qualifier" select="$qualifier"/>
            </xsl:call-template>
          </xsl:if>
        </xsl:for-each>
      </xsl:variable>
      
      <xsl:variable name="service_match">      
        <xsl:for-each select="$services_">
          <xsl:variable name="suffix" select="
            substring($_dds_topic,
            string-length($_dds_topic) - string-length(@suffix) +1)" />
          <xsl:if test="$suffix = @suffix">
            <xsl:variable name="object" select="substring($_dds_topic, 1, 
                  string-length($_dds_topic) - string-length($suffix))" />
            <xsl:call-template name="services">
              <xsl:with-param name="service" select="$object"/>
              <xsl:with-param name="kind" select="$kind"/>
              <xsl:with-param name="type" select="
              $services_[@prefix=$prefix and @suffix=$suffix]/@type"/>
              <xsl:with-param name="qualifier" select="$qualifier"/>
            </xsl:call-template>
          </xsl:if>
        </xsl:for-each>
      </xsl:variable>
      
      <xsl:variable name="topic_match">      
        <xsl:for-each select="$topics_">
          <xsl:call-template name="topics">
            <xsl:with-param name="topic" select="$_dds_topic"/>
            <xsl:with-param name="kind" select="$kind"/>
            <xsl:with-param name="type" select="
            $topics_[@prefix=$prefix and @suffix='']/@type"/>
            <xsl:with-param name="qualifier" select="$qualifier"/>
          </xsl:call-template>
        </xsl:for-each>
      </xsl:variable>

    <xsl:choose>
      <xsl:when test="$action_match != ''">
        <xsl:apply-templates mode="substitute"
          select="ext:node-set($action_match)"/>
      </xsl:when>
      <xsl:when test="$service_match != ''">
        <xsl:apply-templates mode="substitute"
          select="ext:node-set($service_match)"/>
      </xsl:when>
      <xsl:when test="$topic_match != ''">
        <xsl:apply-templates mode="substitute"
          select="ext:node-set($topic_match)"/>
      </xsl:when>
    </xsl:choose>
  </xsl:for-each>
</xsl:template>

<xsl:template match="@*|node()" mode="substitute">
  <xsl:copy>
    <xsl:apply-templates select="@*|node()" mode="substitute"/>
  </xsl:copy>
</xsl:template>


<xsl:template name="actions">
  <xsl:param name="action"/>
  <xsl:param name="kind"/>
  <xsl:param name="type"/>
  <xsl:param name="qualifier"/>

  <xsl:variable name="action_permissions"
  select="document('')/xsl:stylesheet/
    ros:action/permissions/permission" />
  <xsl:variable name="rule" select="
    document('')/xsl:stylesheet/
    ros:action/permissions/permission[@kind=$kind and @type=$type]/@rule"/>

  <actions>
    <xsl:attribute name="{$rule}">
      <xsl:value-of select="$qualifier"/>
    </xsl:attribute>
    <action>
      <xsl:value-of select="$action"/>
    </action>
  </actions>
</xsl:template>


<xsl:template name="services">
  <xsl:param name="service"/>
  <xsl:param name="kind"/>
  <xsl:param name="type"/>
  <xsl:param name="qualifier"/>

  <xsl:variable name="service_permissions"
  select="document('')/xsl:stylesheet/
    ros:service/permissions/permission" />
  <xsl:variable name="rule" select="
    document('')/xsl:stylesheet/
    ros:service/permissions/permission[@kind=$kind and @type=$type]/@rule"/>

  <services>
    <xsl:attribute name="{$rule}">
      <xsl:value-of select="$qualifier"/>
    </xsl:attribute>
    <service>
      <xsl:value-of select="$service"/>
    </service>
  </services>
</xsl:template>


<xsl:template name="topics">
  <xsl:param name="topic"/>
  <xsl:param name="kind"/>
  <xsl:param name="type"/>
  <xsl:param name="qualifier"/>

  <xsl:variable name="topic_permissions"
  select="document('')/xsl:stylesheet/
    ros:topic/permissions/permission" />
  <xsl:variable name="rule" select="
    document('')/xsl:stylesheet/
    ros:topic/permissions/permission[@kind=$kind and @type=$type]/@rule"/>

  <topics>
    <xsl:attribute name="{$rule}">
      <xsl:value-of select="$qualifier"/>
    </xsl:attribute>
    <topic>
      <xsl:value-of select="$topic"/>
    </topic>
  </topics>
</xsl:template>


<xsl:template match="@* | node()" mode="namespace">
  <xsl:copy>
    <xsl:apply-templates select="@* | node()" mode="namespace"/>
  </xsl:copy>
</xsl:template>


<xsl:template match="topic | service | action" mode="namespace">
  <xsl:variable name="ns" select="../../@ns"/>
  <xsl:variable name="node" select="../../@node"/>
  <xsl:variable name="name" select="."/>
  <xsl:variable name="_ns">
    <xsl:call-template name="DelimitNamespace">
      <xsl:with-param name="ns" select="$ns"/>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="_fqn" select="concat($_ns, $node, '/')"/>

  <xsl:copy>
    <xsl:choose>
      <xsl:when test="starts-with($name, $_fqn)">
        <xsl:value-of select="concat('~/', substring-after($name, $_fqn))"/>
      </xsl:when>
      <xsl:when test="starts-with($name, $_ns)">
        <xsl:value-of select="substring-after($name, $_ns)"/>
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="$name"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:copy>
</xsl:template>


<xsl:template name="DelimitNamespace">
  <xsl:param name="ns"/>
  <xsl:choose>
    <xsl:when test="substring($ns, string-length($ns), 1) = '/'">
      <xsl:value-of select="$ns"/>
    </xsl:when>
    <xsl:otherwise>
      <xsl:value-of select="concat($ns, '/')"/>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>


<xsl:template match="@*|node()" mode="sort">
  <xsl:copy>
    <xsl:apply-templates select="@*" mode="sort">
      <xsl:sort select="local-name()"/>
      <xsl:sort select="."/>
    </xsl:apply-templates>
    <xsl:apply-templates select="node()" mode="sort">
      <xsl:sort select="local-name()"/>
      <xsl:sort select="local:key2(.)"/>
      <xsl:sort select="local:key3(.)"/>
      <xsl:sort select="." data-type="number"/>
    </xsl:apply-templates>
  </xsl:copy>
</xsl:template>

<func:function name="local:key2" mode="sort">
  <xsl:param name="e" select="."/>

  <func:result>
    <xsl:for-each select="$e/@*">
      <xsl:sort select="local-name()"/>
      <xsl:sort select="string()"/>
      <xsl:value-of select="concat(local-name(), ' ')"/>
    </xsl:for-each>
  </func:result>
</func:function>

<func:function name="local:key3" mode="sort">
  <xsl:param name="e" select="."/>
  <func:result>
    <xsl:for-each select="$e/@*">
      <xsl:sort select="local-name()"/>
      <xsl:sort select="string()"/>
      <xsl:value-of select="concat(string(), ' ')"/>
    </xsl:for-each>
  </func:result>
</func:function>

</xsl:stylesheet>
