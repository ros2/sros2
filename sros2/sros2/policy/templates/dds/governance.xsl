<?xml version="1.0" encoding="UTF-8"?>

<!-- TODO Sort resulting rules? -->
<xsl:stylesheet version="1.0"
 xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
 xmlns:ext="http://exslt.org/common" exclude-result-prefixes="ext">
 <xsl:output omit-xml-declaration="yes" indent="yes"/>
 <xsl:strip-space elements="*"/>


<xsl:variable name="template_domains">
  <domains>
    <id>0</id>
  </domains>
</xsl:variable>

<xsl:template match="/policy/profiles">
  <xsl:variable name="dds">
    <dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_governance.xsd">
      <domain_access_rules>
        <domain_rule>
          <!-- FIXME what to do if multiple profiles? -->
          <xsl:for-each select="profile">
            <xsl:variable name="_ns">
              <xsl:call-template name="DelimitNamespace">
                <xsl:with-param name="ns" select="@ns"/>
              </xsl:call-template>
            </xsl:variable>
            <xsl:copy-of select="$template_domains"/>
            <allow_unauthenticated_participants>false</allow_unauthenticated_participants>
            <enable_join_access_control>true</enable_join_access_control>
            <discovery_protection_kind>SIGN</discovery_protection_kind>
            <liveliness_protection_kind>SIGN</liveliness_protection_kind>
            <rtps_protection_kind>SIGN</rtps_protection_kind>
            <topic_access_rules>

            <xsl:for-each select="./*[@* = 'ALLOW']">
              <xsl:call-template name="TranslatePermissions">
                <!--xsl:with-param name="qualifier" select="'ALLOW'"/-->
              </xsl:call-template>
            </xsl:for-each>
            </topic_access_rules>
          </xsl:for-each>
        </domain_rule>
      </domain_access_rules>
    </dds>
  </xsl:variable>

 <xsl:apply-templates mode="sort"
   select="ext:node-set($dds)"/>
</xsl:template>

<xsl:template name="TranslatePermissions">
  <xsl:param name="qualifier"/>
  <xsl:apply-templates select="."/>
</xsl:template>

<xsl:template name="EnableProtectionsDefault">
  <enable_discovery_protection>true</enable_discovery_protection>
  <enable_liveliness_protection>true</enable_liveliness_protection>
  <enable_read_access_control>true</enable_read_access_control>
  <enable_write_access_control>true</enable_write_access_control>
</xsl:template>

<xsl:template match="topics">
  <xsl:for-each select="topic">
  <xsl:variable name="fqn">
    <xsl:apply-templates select="."/>
  </xsl:variable>
  <topic_rule>
    <topic_expression>rt<xsl:value-of select="$fqn"/></topic_expression>
    <xsl:call-template name="EnableProtectionsDefault"/>
    <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
    <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
  </topic_rule>
  </xsl:for-each>
</xsl:template>

<xsl:template match="services">
  <xsl:for-each select="service">
    <xsl:variable name="fqn">
      <xsl:apply-templates select="."/>
    </xsl:variable>
    <topic_rule>
      <topic_expression>rq<xsl:value-of select="$fqn"/>Request</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rr<xsl:value-of select="$fqn"/>Reply</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
  </xsl:for-each>
</xsl:template>

<xsl:template match="actions">
  <xsl:for-each select="action">
    <xsl:variable name="fqn">
      <xsl:apply-templates select="."/>
    </xsl:variable>
    <topic_rule>
      <topic_expression>rq<xsl:value-of select="$fqn"/>/_action/cancel_goalRequest</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rr<xsl:value-of select="$fqn"/>/_action/cancel_goalReply</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rq<xsl:value-of select="$fqn"/>/_action/get_resultRequest</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rr<xsl:value-of select="$fqn"/>/_action/get_resultReply</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rq<xsl:value-of select="$fqn"/>/_action/send_goalRequest</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rr<xsl:value-of select="$fqn"/>/_action/send_goalReply</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rt<xsl:value-of select="$fqn"/>/_action/feedback</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
    <topic_rule>
      <topic_expression>rt<xsl:value-of select="$fqn"/>/_action/status</topic_expression>
      <xsl:call-template name="EnableProtectionsDefault"/>
      <metadata_protection_kind><xsl:value-of select="../@protection"/></metadata_protection_kind>
      <data_protection_kind><xsl:value-of select="../@protection"/></data_protection_kind>
    </topic_rule>
  </xsl:for-each>
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
      <xsl:value-of select="concat($_ns, $node, '/', $_name)"/>
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

<xsl:template match="@*|node()" mode="sort">
  <xsl:copy>
    <xsl:apply-templates select="@*|node()" mode="sort"/>
  </xsl:copy>
</xsl:template>

</xsl:stylesheet>
