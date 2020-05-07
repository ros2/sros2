<?xml version="1.0" encoding="UTF-8"?>

<xsl:stylesheet version="1.0"
 xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
 xmlns:ext="http://exslt.org/common" exclude-result-prefixes="ext">
 <xsl:output omit-xml-declaration="yes" indent="yes"/>
 <xsl:strip-space elements="*"/>

<xsl:param name="not_valid_before" select="'2020-05-01T00:00:00'"/>
<xsl:param name="not_valid_after" select="'2030-05-01T00:00:00'"/>

<xsl:variable name="template_validity">
  <validity>
    <not_before><xsl:value-of select="$not_valid_before" /></not_before>
    <not_after><xsl:value-of select="$not_valid_after" /></not_after>
  </validity>
</xsl:variable>

<xsl:variable name="template_domains">
  <domains>
    <id>0</id>
  </domains>
</xsl:variable>

<xsl:param name="allow_ros_discovery_topic" select="0"/>

<xsl:template match="/policy/enclaves">
  <xsl:variable name="dds">
    <dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd">
      <permissions>
        <xsl:for-each select="enclave">
          <xsl:variable name="common_name">
            <xsl:value-of select="@path"/>
          </xsl:variable>
          <xsl:for-each select="profiles">
            <grant name="{$common_name}">
              <subject_name>CN=<xsl:value-of select="$common_name"/></subject_name>
              <xsl:copy-of select="$template_validity"/>
              <xsl:if test="./profile/*[@*='DENY']">
                <deny_rule>
                  <xsl:copy-of select="$template_domains"/>
                  <xsl:for-each select="./profile">
                    <xsl:for-each select="./*[@* = 'DENY']">
                      <xsl:call-template name="TranslatePermissions">
                        <xsl:with-param name="qualifier" select="'DENY'"/>
                      </xsl:call-template>
                    </xsl:for-each>
                  </xsl:for-each>
                </deny_rule>
              </xsl:if>
              <xsl:if test="./profile/*[@* = 'ALLOW']">
                <allow_rule>
                  <xsl:copy-of select="$template_domains"/>
                  <xsl:for-each select="./profile">
                    <xsl:for-each select="./*[@* = 'ALLOW']">
                      <xsl:call-template name="TranslatePermissions">
                        <xsl:with-param name="qualifier" select="'ALLOW'"/>
                      </xsl:call-template>
                    </xsl:for-each>
                  </xsl:for-each>
                </allow_rule>
              </xsl:if>
              <xsl:if test="$allow_ros_discovery_topic">
                <allow_rule>
                  <xsl:copy-of select="$template_domains"/>
                  <publish>
                    <topics>
                      <topic>ros_discovery_info</topic>
                    </topics>
                  </publish>
                  <subscribe>
                    <topics>
                      <topic>ros_discovery_info</topic>
                    </topics>
                  </subscribe>
                </allow_rule>
              </xsl:if>
              <default>DENY</default>
            </grant>
          </xsl:for-each>
        </xsl:for-each>
      </permissions>
    </dds>
  </xsl:variable>

  <xsl:variable name="dds_sorted">
    <xsl:apply-templates mode="sort"
      select="ext:node-set($dds)"/>
  </xsl:variable>
   
 <xsl:apply-templates mode="prune"
   select="ext:node-set($dds_sorted)"/>
</xsl:template>

<xsl:template name="TranslatePermissions">
  <xsl:param name="qualifier"/>
  <xsl:if test="@publish = $qualifier">
    <xsl:apply-templates select="." mode="publish"/>
  </xsl:if>
  <xsl:if test="@subscribe = $qualifier">
    <xsl:apply-templates select="." mode="subscribe"/>
  </xsl:if>
  <xsl:if test="@request = $qualifier">
    <xsl:apply-templates select="." mode="request"/>
  </xsl:if>
  <xsl:if test="@reply = $qualifier">
    <xsl:apply-templates select="." mode="reply"/>
  </xsl:if>
  <xsl:if test="@call = $qualifier">
    <xsl:apply-templates select="." mode="call"/>
  </xsl:if>
  <xsl:if test="@execute = $qualifier">
    <xsl:apply-templates select="." mode="execute"/>
  </xsl:if>
</xsl:template>

<xsl:template match="topics" mode="publish">
  <publish>
    <topics>
      <xsl:for-each select="topic">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rt<xsl:value-of select="$fqn"/></topic>
      </xsl:for-each>
    </topics>
  </publish>
</xsl:template>

<xsl:template match="topics" mode="subscribe">
  <subscribe>
    <topics>
      <xsl:for-each select="topic">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rt<xsl:value-of select="$fqn"/></topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="services" mode="request">
  <publish>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>Request</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>Reply</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="services" mode="reply">
  <publish>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>Reply</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>Request</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="actions" mode="call">
  <publish>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/cancel_goalRequest</topic>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/get_resultRequest</topic>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/send_goalRequest</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/cancel_goalReply</topic>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/get_resultReply</topic>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/send_goalReply</topic>
        <topic>rt<xsl:value-of select="$fqn"/>/_action/feedback</topic>
        <topic>rt<xsl:value-of select="$fqn"/>/_action/status</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="actions" mode="execute">
  <publish>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/cancel_goalReply</topic>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/get_resultReply</topic>
        <topic>rr<xsl:value-of select="$fqn"/>/_action/send_goalReply</topic>
        <topic>rt<xsl:value-of select="$fqn"/>/_action/feedback</topic>
        <topic>rt<xsl:value-of select="$fqn"/>/_action/status</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/cancel_goalRequest</topic>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/get_resultRequest</topic>
        <topic>rq<xsl:value-of select="$fqn"/>/_action/send_goalRequest</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="topic | service | action">
  <xsl:variable name="ns" select="../../@ns"/>
  <xsl:variable name="node" select="../../@node"/>
  <xsl:variable name="name" select="."/>
  <xsl:choose>
    <xsl:when test="substring($name, 1, 1) = '/'">
      <xsl:value-of select="$name"/>
    </xsl:when>
    <xsl:when test="substring($name, 1, 1) = '~'">
      <xsl:variable name="_ns">
        <xsl:call-template name="DelimitNamespace">
          <xsl:with-param name="ns" select="$ns"/>
        </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="_name" select="substring($name, 2)"/>
      <xsl:value-of select="concat($_ns, $node, $_name)"/>
    </xsl:when>
    <xsl:otherwise>
      <xsl:variable name="_ns">
        <xsl:call-template name="DelimitNamespace">
          <xsl:with-param name="ns" select="$ns"/>
        </xsl:call-template>
      </xsl:variable>
      <xsl:value-of select="concat($_ns, $name)"/>
    </xsl:otherwise>
  </xsl:choose>
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

<xsl:template match="allow_rule|deny_rule" mode="sort">
  <xsl:element  name="{local-name()}">
    <xsl:apply-templates select="domains"  mode="sort"/>
    <xsl:if test="publish">
      <publish>
        <topics>
          <xsl:for-each select="publish/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()" mode="sort"/>
          </xsl:for-each>
        </topics>
      </publish>
    </xsl:if>
    <xsl:if test="subscribe">
      <subscribe>
        <topics>
          <xsl:for-each select="subscribe/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()" mode="sort"/>
          </xsl:for-each>
        </topics>
      </subscribe>
    </xsl:if>
  </xsl:element>
</xsl:template>

<xsl:template match="@*|node()" mode="sort">
  <xsl:copy>
    <xsl:apply-templates select="@*|node()" mode="sort"/>
  </xsl:copy>
</xsl:template>

<xsl:template match="topic[. = preceding-sibling::topic]" mode="prune"/>

<xsl:template match="@*|node()" mode="prune">
  <xsl:copy>
    <xsl:apply-templates select="@* | node()" mode="prune"/>
  </xsl:copy>
</xsl:template>

</xsl:stylesheet>
